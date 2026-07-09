"""Jet engine — modular procedural template.

Implements the reviewed modular spec
``articraft_template_authoring/specs_modular_v1/jet_engine.md``.

Category identity (the rule the deleted template violated): a jet engine ALWAYS
has at least one rotor spinning CONTINUOUSly about the engine longitudinal axis
(world +X here). Single-rotor bodies emit one ``*_spin`` CONTINUOUS joint;
``cutaway_twospool`` emits two independent CONTINUOUS spools (fan + core).

Slots (manual dispatch — a static ``outer_shell`` is the structural hub, with
the rotor(s), nozzle petals and access door hung off it as parallel children):

    A mount_platform : bare_inline | ground_display_stand | top_pylon_mount
    B engine_core    : smooth_turbofan | turbojet_module | cutaway_twospool
    C aft_nozzle     : static_exhaust_cone | variable_petal_nozzle | static_chevron_ring
    D access_door    : none | belly_fold_down_panel | side_equipment_bay_door

Multiplicities (see spec §Multiplicity):
    M1 petal_count (joint-bearing, REVOLUTE per petal)
    M2 fan_blade_count / M3 compressor_stage_count / M4 turbine_stage_count
       (jointless — repeated visuals inside a single spinning part)

Adopted 5-star sources (see spec Module Source Index): S1 ec62c1e0, S2 0001,
S6 278c04, S8 4f9be8, S14 a602a9, S15 ad0120, S18 e2123c, S19 ec6fd5.

Coordinate convention
---------------------
Engine axis = world +X. The ``outer_shell`` local frame is built axially
centred (x in ``[-EL/2, +EL/2]``). The engine centreline sits at local height
``axis_z`` so the *belly* of the shell can be the part's local origin when it is
cradled on a stand, and the *crown* can be the origin when it hangs from a
pylon. This makes the FIXED mount joint land on real geometry on both sides
(``fail_if_articulation_origin_far_from_geometry`` measures the joint origin
against the parent geometry and the child's local origin against the child).

LatheGeometry / FanRotorGeometry are authored about local Z then rotated
``rotate_y(pi/2)`` onto +X; radial blades / struts / petals are arrayed in the
Y-Z plane about ``axis_z``.
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
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)

__modular__ = True

# --------------------------------------------------------------------------- #
# Slot enums
# --------------------------------------------------------------------------- #

MountPlatform = Literal["bare_inline", "ground_display_stand", "top_pylon_mount"]
EngineCore = Literal["smooth_turbofan", "turbojet_module", "cutaway_twospool"]
AftNozzle = Literal["static_exhaust_cone", "variable_petal_nozzle", "static_chevron_ring"]
AccessDoor = Literal["none", "belly_fold_down_panel", "side_equipment_bay_door"]
PetalGangMode = Literal["mimic", "independent"]
PaletteTheme = Literal["service_white", "military_gray", "carrier_navy", "test_cell_orange"]

MOUNT_PLATFORMS: tuple[MountPlatform, ...] = (
    "bare_inline",
    "ground_display_stand",
    "top_pylon_mount",
)
ENGINE_CORES: tuple[EngineCore, ...] = ("smooth_turbofan", "turbojet_module", "cutaway_twospool")
AFT_NOZZLES: tuple[AftNozzle, ...] = (
    "static_exhaust_cone",
    "variable_petal_nozzle",
    "static_chevron_ring",
)
ACCESS_DOORS: tuple[AccessDoor, ...] = (
    "none",
    "belly_fold_down_panel",
    "side_equipment_bay_door",
)

PALETTES: dict[str, dict[str, tuple[float, float, float, float]]] = {
    "service_white": {
        "nacelle": (0.80, 0.82, 0.85, 1.0),
        "metal": (0.62, 0.66, 0.72, 1.0),
        "dark": (0.24, 0.26, 0.29, 1.0),
        "hot": (0.56, 0.45, 0.34, 1.0),
        "blade": (0.50, 0.53, 0.58, 1.0),
        "accent": (0.70, 0.16, 0.10, 1.0),
    },
    "military_gray": {
        "nacelle": (0.44, 0.46, 0.43, 1.0),
        "metal": (0.50, 0.52, 0.54, 1.0),
        "dark": (0.17, 0.18, 0.19, 1.0),
        "hot": (0.45, 0.37, 0.29, 1.0),
        "blade": (0.40, 0.42, 0.44, 1.0),
        "accent": (0.68, 0.55, 0.15, 1.0),
    },
    "carrier_navy": {
        "nacelle": (0.24, 0.30, 0.38, 1.0),
        "metal": (0.52, 0.56, 0.61, 1.0),
        "dark": (0.12, 0.14, 0.17, 1.0),
        "hot": (0.50, 0.40, 0.31, 1.0),
        "blade": (0.36, 0.40, 0.46, 1.0),
        "accent": (0.82, 0.72, 0.20, 1.0),
    },
    "test_cell_orange": {
        "nacelle": (0.88, 0.52, 0.18, 1.0),
        "metal": (0.70, 0.72, 0.75, 1.0),
        "dark": (0.20, 0.20, 0.22, 1.0),
        "hot": (0.60, 0.47, 0.33, 1.0),
        "blade": (0.58, 0.60, 0.63, 1.0),
        "accent": (0.20, 0.22, 0.26, 1.0),
    },
}


# --------------------------------------------------------------------------- #
# Config
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class JetEngineConfig:
    mount_platform: MountPlatform | None = None
    engine_core: EngineCore | None = None
    aft_nozzle: AftNozzle | None = None
    access_door: AccessDoor | None = None
    petal_count: int = 12
    petal_gang_mode: PetalGangMode = "independent"
    fan_blade_count: int = 20
    compressor_stage_count: int = 5
    turbine_stage_count: int = 2
    engine_length_scale: float = 1.0
    nacelle_diameter_scale: float = 1.0
    stand_height_scale: float = 1.0
    palette_theme: PaletteTheme = "service_white"


@dataclass(frozen=True)
class ResolvedJetEngineConfig:
    mount_platform: MountPlatform
    engine_core: EngineCore
    aft_nozzle: AftNozzle
    access_door: AccessDoor
    petal_count: int
    petal_gang_mode: PetalGangMode
    fan_blade_count: int
    compressor_stage_count: int
    turbine_stage_count: int
    engine_length: float
    nacelle_radius: float  # max outer radius of the static shell
    stand_height: float  # engine belly height above the ground / below the pylon foot
    axis_z: float  # engine centreline height in the outer_shell local frame
    palette_theme: PaletteTheme
    palette: dict[str, tuple[float, float, float, float]]


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(v)))


def _clampi(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(v)))


# --------------------------------------------------------------------------- #
# Procedural sampling (seed 0 is NOT special — Contract 4)
# --------------------------------------------------------------------------- #


def config_from_seed(seed: int) -> JetEngineConfig:
    rng = random.Random(seed)
    # Turbofans dominate the 5-star pool; weight accordingly but keep all cores
    # well represented for topology diversity.
    engine_core: EngineCore = rng.choices(ENGINE_CORES, weights=(0.45, 0.27, 0.28), k=1)[0]

    # Compatibility gating (see spec audit + resolve_config):
    #  - a bare turbojet has no high-bypass chevron tail and no nacelle
    #    belly/bay for a service door;
    #  - the cutaway exposes the core and always ends in a centre-body plug.
    if engine_core == "turbojet_module":
        aft_nozzle: AftNozzle = rng.choice(("static_exhaust_cone", "variable_petal_nozzle"))
        access_door: AccessDoor = "none"
    elif engine_core == "cutaway_twospool":
        aft_nozzle = "static_exhaust_cone"
        access_door = rng.choice(ACCESS_DOORS)
    else:
        aft_nozzle = rng.choice(AFT_NOZZLES)
        access_door = rng.choice(ACCESS_DOORS)

    return JetEngineConfig(
        mount_platform=rng.choice(MOUNT_PLATFORMS),
        engine_core=engine_core,
        aft_nozzle=aft_nozzle,
        access_door=access_door,
        petal_count=rng.randint(6, 18),
        petal_gang_mode=rng.choice(("mimic", "independent")),
        fan_blade_count=rng.randint(12, 28),
        compressor_stage_count=rng.randint(3, 8),
        turbine_stage_count=rng.randint(2, 5),
        engine_length_scale=round(rng.uniform(0.85, 1.20), 4),
        nacelle_diameter_scale=round(rng.uniform(0.85, 1.20), 4),
        stand_height_scale=round(rng.uniform(0.80, 1.25), 4),
        palette_theme=rng.choice(tuple(PALETTES.keys())),
    )


def resolve_config(config: JetEngineConfig) -> ResolvedJetEngineConfig:
    mount = config.mount_platform or "bare_inline"
    core = config.engine_core or "smooth_turbofan"
    nozzle = config.aft_nozzle or "static_exhaust_cone"
    door = config.access_door or "none"
    if mount not in MOUNT_PLATFORMS:
        raise ValueError(f"Unsupported mount_platform: {mount!r}")
    if core not in ENGINE_CORES:
        raise ValueError(f"Unsupported engine_core: {core!r}")
    if nozzle not in AFT_NOZZLES:
        raise ValueError(f"Unsupported aft_nozzle: {nozzle!r}")
    if door not in ACCESS_DOORS:
        raise ValueError(f"Unsupported access_door: {door!r}")
    if config.palette_theme not in PALETTES:
        raise ValueError(f"Unsupported palette_theme: {config.palette_theme!r}")

    # Compatibility downgrades — keep the build legal regardless of input config.
    if core == "turbojet_module":
        if nozzle == "static_chevron_ring":
            nozzle = "static_exhaust_cone"
        door = "none"
    # A ground stand cradles the belly, so a belly fold-down panel would collide
    # with the saddle — route it to the side bay instead (keeps a door slot).
    if mount == "ground_display_stand" and door == "belly_fold_down_panel":
        door = "side_equipment_bay_door"
    # The cutaway exposes the core, so it always terminates in an on-axis
    # centre-body exhaust plug. Chevron / variable-petal nozzles need a closed
    # rear casing to mount on and are reserved for the turbofan / turbojet shells
    # (documented compatibility gate; narrows cutaway's nozzle axis to 1).
    if core == "cutaway_twospool":
        nozzle = "static_exhaust_cone"

    engine_length = _clamp(2.0 * config.engine_length_scale, 1.55, 2.55)
    nacelle_radius = _clamp(0.62 * config.nacelle_diameter_scale, 0.48, 0.82)
    # Effective outer radius of the static shell envelope (used for the mount drop).
    shell_outer_r = nacelle_radius * (0.78 if core == "turbojet_module" else 1.0)
    if mount == "ground_display_stand":
        axis_z = shell_outer_r  # centreline above the belly; local origin at the belly
    elif mount == "top_pylon_mount":
        axis_z = -shell_outer_r  # centreline below the crown; local origin at the crown
    else:
        axis_z = 0.0

    return ResolvedJetEngineConfig(
        mount_platform=mount,
        engine_core=core,
        aft_nozzle=nozzle,
        access_door=door,
        petal_count=_clampi(config.petal_count, 6, 18),
        petal_gang_mode=config.petal_gang_mode,
        fan_blade_count=_clampi(config.fan_blade_count, 12, 28),
        compressor_stage_count=_clampi(config.compressor_stage_count, 3, 8),
        turbine_stage_count=_clampi(config.turbine_stage_count, 2, 5),
        engine_length=engine_length,
        nacelle_radius=nacelle_radius,
        stand_height=_clamp(0.95 * config.stand_height_scale, 0.65, 1.35),
        axis_z=axis_z,
        palette_theme=config.palette_theme,
        palette=dict(PALETTES[config.palette_theme]),
    )


# --------------------------------------------------------------------------- #
# Geometry helpers (engine axis = world +X)
# --------------------------------------------------------------------------- #


def _cyl_x() -> tuple[float, float, float]:
    """rpy that lays a Cylinder (default axis +Z) along world +X."""
    return (0.0, math.pi / 2.0, 0.0)


def _cyl_y() -> tuple[float, float, float]:
    """rpy that lays a Cylinder (default axis +Z) along world +Y."""
    return (math.pi / 2.0, 0.0, 0.0)


def _to_x(geom: MeshGeometry) -> MeshGeometry:
    """Rotate a Z-axis lathe/fan mesh onto world +X."""
    return geom.rotate_y(math.pi / 2.0)


def _mesh(model: ArticulatedObject, geom: MeshGeometry, name: str):
    if model.assets is not None:
        return mesh_from_geometry(geom, model.assets.mesh_path(f"{name}.obj"))
    return mesh_from_geometry(geom, name)


def _shell_outer_r(r: ResolvedJetEngineConfig) -> float:
    return r.nacelle_radius * (0.78 if r.engine_core == "turbojet_module" else 1.0)


# Shell radial profiles, expressed as (axial_frac in [0,1], radius_frac of NR).
# Authored after S2 0001 / S6 278c04 / S19 ec6fd5 nacelle & casing lathes. The
# front bore (inner radius near frac 0) is kept wide — close to the fan radius —
# so the open intake shows the fan face instead of a narrow lip closing over it.
_TURBOFAN_OUTER = [
    (0.00, 0.86),
    (0.08, 1.00),
    (0.32, 1.00),
    (0.72, 0.90),
    (1.00, 0.76),
]
_TURBOFAN_INNER = [
    (0.00, 0.80),
    (0.08, 0.90),
    (0.32, 0.90),
    (0.72, 0.78),
    (1.00, 0.64),
]
_CASING_OUTER = [(0.00, 0.92), (0.08, 1.00), (0.85, 1.00), (1.00, 0.84)]
_CASING_INNER = [(0.00, 0.82), (0.08, 0.90), (0.85, 0.90), (1.00, 0.76)]


def _scaled_profile(
    points: list[tuple[float, float]], length: float, radius: float, *, x0: float
) -> list[tuple[float, float]]:
    """(zfrac, rfrac) -> (r, z) with z mapped to ``x0 + zfrac * length``."""
    return [(rf * radius, x0 + zf * length) for zf, rf in points]


def _shell_radius_at(r: ResolvedJetEngineConfig, frac: float, *, inner: bool = False) -> float:
    """Interpolate the shell outer (or inner) radius at axial fraction ``frac``."""
    if r.engine_core == "turbojet_module":
        prof = _CASING_INNER if inner else _CASING_OUTER
        scale = r.nacelle_radius * 0.78
    else:
        prof = _TURBOFAN_INNER if inner else _TURBOFAN_OUTER
        scale = r.nacelle_radius
    frac = max(0.0, min(1.0, frac))
    for (z0, r0), (z1, r1) in zip(prof, prof[1:]):
        if z0 <= frac <= z1:
            t = 0.0 if z1 == z0 else (frac - z0) / (z1 - z0)
            return (r0 + (r1 - r0) * t) * scale
    return prof[-1][1] * scale


def _shell_radius_at_x(r: ResolvedJetEngineConfig, x: float, *, inner: bool = False) -> float:
    """Shell radius at world axial ``x`` (handles the compressed cutaway cowl)."""
    front = -0.5 * r.engine_length
    shell_len = r.engine_length * (0.46 if r.engine_core == "cutaway_twospool" else 1.0)
    return _shell_radius_at(r, (x - front) / shell_len, inner=inner)


def _build_shell_lathe(
    model: ArticulatedObject, r: ResolvedJetEngineConfig, *, name: str, length_frac: float = 1.0
):
    EL = r.engine_length
    length = EL * length_frac
    x0 = -EL * 0.5
    if r.engine_core == "turbojet_module":
        outer, inner, scale = _CASING_OUTER, _CASING_INNER, r.nacelle_radius * 0.78
        lip = 6
    else:
        outer, inner, scale = _TURBOFAN_OUTER, _TURBOFAN_INNER, r.nacelle_radius
        lip = 8
    geom = LatheGeometry.from_shell_profiles(
        _scaled_profile(outer, length, scale, x0=x0),
        _scaled_profile(inner, length, scale, x0=x0),
        segments=48,
        start_cap="round",
        end_cap="flat",
        lip_samples=lip,
    )
    return _mesh(model, _to_x(geom), name)


def _lathe_solid(
    model: ArticulatedObject,
    points: list[tuple[float, float]],
    length: float,
    radius: float,
    *,
    x0: float,
    name: str,
    segments: int = 48,
):
    geom = LatheGeometry(_scaled_profile(points, length, radius, x0=x0), segments=segments)
    return _mesh(model, _to_x(geom), name)


def _ring(
    model: ArticulatedObject,
    *,
    inner_r: float,
    outer_r: float,
    width: float,
    name: str,
    segments: int = 48,
):
    """Hollow annular ring band about the engine axis (open centre for flow)."""
    geom = LatheGeometry.from_shell_profiles(
        [(outer_r, -width * 0.5), (outer_r, width * 0.5)],
        [(inner_r, -width * 0.5), (inner_r, width * 0.5)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return _mesh(model, _to_x(geom), name)


def _radial_pattern(base: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    out = MeshGeometry()
    for i in range(count):
        out.merge(base.copy().rotate_z(angle_offset + i * math.tau / count))
    return out


def _blade_section(
    radius: float, lean: float, chord: float, thickness: float
) -> list[tuple[float, float, float]]:
    """4-point airfoil loop at a given span ``radius`` (x), chord along z,
    thickness along y — adapted from S19 ec6fd5 ``_blade_section``."""
    h = thickness * 0.5
    return [
        (radius, lean - 0.95 * h, -0.52 * chord),
        (radius, lean + 0.18 * h, -0.10 * chord),
        (radius, lean + 1.00 * h, 0.48 * chord),
        (radius, lean - 0.30 * h, 0.12 * chord),
    ]


def _blade_disk(
    model: ArticulatedObject,
    *,
    hub_r: float,
    outer_r: float,
    blade_count: int,
    name: str,
    chord_frac: float = 0.42,
) -> MeshGeometry:
    """A spinning blade row: one lofted airfoil swept root->tip, arrayed N times
    about the engine axis. Pure-python (section_loft) — the fast, faithful blade
    primitive the spec endorses (no cadquery boolean unions)."""
    span = max(outer_r - hub_r, 1e-3)
    chord = max(span * chord_frac, 0.02)
    th = max(chord * 0.16, 0.006)
    blade = repair_loft(
        section_loft(
            [
                _blade_section(hub_r, -th * 0.4, chord, th),
                _blade_section(hub_r + 0.5 * span, 0.0, chord * 0.85, th * 0.7),
                _blade_section(outer_r, th * 0.5, chord * 0.45, th * 0.45),
            ]
        )
    )
    return _mesh(model, _to_x(_radial_pattern(blade, max(2, int(blade_count)))), name)


def _petal_mesh(
    model: ArticulatedObject,
    *,
    length: float,
    w_root: float,
    w_tip: float,
    thickness: float,
    name: str,
) -> MeshGeometry:
    """A convergent-nozzle petal: a shallow concave flap that tapers from root
    to tip, lofted through 3 stations (S18 e2123c / S8 4f9be8 petal skins).
    Built along local +x (aft) from the hinge at x=0."""

    def sec(x: float, w: float, t: float) -> list[tuple[float, float, float]]:
        # x = axial station, y = radial (concave), z = tangential width
        return [
            (x, -t * 0.45, -w * 0.5),
            (x, t * 0.55, -w * 0.34),
            (x, t * 0.55, w * 0.34),
            (x, -t * 0.45, w * 0.5),
        ]

    geom = repair_loft(
        section_loft(
            [
                sec(0.0, w_root, thickness),
                sec(length * 0.55, 0.5 * (w_root + w_tip), thickness * 0.82),
                sec(length, w_tip, thickness * 0.6),
            ]
        )
    )
    return _mesh(model, geom, name)


def _radial_struts(
    shell,
    r: ResolvedJetEngineConfig,
    *,
    x_pos: float,
    inner_r: float,
    outer_r: float,
    count: int,
    name: str,
    embed: float | None = None,
) -> None:
    """Radial vanes/struts tying a central body (inner_r) to the shell wall
    (outer_r). They are deliberately over-long by ``embed`` at both ends so the
    part stays a single connected geometry island."""
    em = (r.nacelle_radius * 0.05) if embed is None else embed
    lo = max(0.0, inner_r - em)
    hi = outer_r + em
    length = hi - lo
    mid = 0.5 * (lo + hi)
    az = r.axis_z
    for i in range(count):
        ang = i * math.tau / count
        shell.visual(
            Box((r.engine_length * 0.05, length, r.nacelle_radius * 0.05)),
            origin=Origin(
                xyz=(x_pos, math.cos(ang) * mid, az + math.sin(ang) * mid),
                rpy=(ang, 0.0, 0.0),
            ),
            material="metal",
            name=f"{name}_{i}",
        )


# --------------------------------------------------------------------------- #
# Slot B — engine_core (static outer_shell + spinning rotor(s))
# --------------------------------------------------------------------------- #


def _build_front_rotor(
    model: ArticulatedObject,
    r: ResolvedJetEngineConfig,
    *,
    part_name: str,
    joint_name: str,
    fan_x: float,
    fan_r: float,
    hub_r: float,
    hub_style: str,
    effort: float,
    velocity: float,
    blade_count: int,
) -> None:
    """Build a single spinning rotor part captured on the engine centreline.

    The part's local frame is the spin joint origin, so its visuals straddle
    local (0,0,0): the hub cylinder sits on the axis there (satisfying the
    joint-origin/child check), the spinner pokes forward, the blade disk fans
    out radially.  Source: S19 ec6fd5 fan_rotor / S2 0001 fan_rotor.
    """
    EL = r.engine_length
    rotor = model.part(part_name)
    if hub_style == "spinner":
        rotor.visual(
            _lathe_solid(
                model,
                [(0.0, 0.0), (0.45, 0.55), (0.78, 0.86), (1.0, 1.0)],
                EL * 0.10,
                hub_r,
                x0=-EL * 0.10,
                name=f"{part_name}_spinner",
            ),
            material="metal",
            name="spinner",
        )
    rotor.visual(
        Cylinder(radius=hub_r, length=EL * 0.12),
        origin=Origin(xyz=(EL * 0.01, 0.0, 0.0), rpy=_cyl_x()),
        material="dark",
        name="fan_hub",
    )
    rotor.visual(
        _blade_disk(
            model,
            hub_r=hub_r,
            outer_r=fan_r,
            blade_count=blade_count,
            name=f"{part_name}_disk",
        ),
        origin=Origin(xyz=(EL * 0.01, 0.0, 0.0)),
        material="dark",  # dark fan reads against the bright open intake
        name="fan_blades",
    )
    # Rearward shaft so the rotor reaches its captured bearing on the shell.
    rotor.visual(
        Cylinder(radius=hub_r * 0.55, length=EL * 0.16),
        origin=Origin(xyz=(EL * 0.12, 0.0, 0.0), rpy=_cyl_x()),
        material="metal",
        name="rotor_shaft",
    )
    model.articulation(
        joint_name,
        ArticulationType.CONTINUOUS,
        parent=model.get_part("outer_shell"),
        child=rotor,
        origin=Origin(xyz=(fan_x, 0.0, r.axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=effort, velocity=velocity),
    )


def _add_central_spine(
    shell,
    model: ArticulatedObject,
    r: ResolvedJetEngineConfig,
    *,
    fan_x: float,
    hub_r: float,
    spine_radius: float,
    spine_rear: float,
) -> None:
    """Static central bearing + core body.

    A short on-axis ``bearing_hub`` straddles the spin joint origin at ``fan_x``
    (so the joint origin lands on parent geometry and the rotor shaft is
    captured), then a tapering ``core_body`` lathe runs aft as the engine core.
    """
    EL = r.engine_length
    az = r.axis_z
    shell.visual(
        Cylinder(radius=hub_r * 0.92, length=EL * 0.22),
        origin=Origin(xyz=(fan_x + EL * 0.07, 0.0, az), rpy=_cyl_x()),
        material="dark",
        name="bearing_hub",
    )
    core_front = fan_x + EL * 0.12
    shell.visual(
        _lathe_solid(
            model,
            [(0.0, 0.0), (0.10, 0.85), (0.55, 1.0), (0.85, 0.72), (1.0, 0.30)],
            spine_rear - core_front,
            spine_radius,
            x0=core_front,
            name="jet_core_body",
            segments=44,
        ),
        origin=Origin(xyz=(0.0, 0.0, az)),
        material="metal",
        name="core_body",
    )


def _build_smooth_turbofan(model: ArticulatedObject, r: ResolvedJetEngineConfig) -> None:
    EL, NR = r.engine_length, r.nacelle_radius
    front = -EL * 0.5
    shell = model.part("outer_shell")
    shell.visual(
        _build_shell_lathe(model, r, name="jet_nacelle_shell"),
        origin=Origin(xyz=(0.0, 0.0, r.axis_z)),
        material="nacelle",
        name="nacelle_shell",
    )

    fan_x = front + 0.12 * EL  # fan just inside the open intake lip
    hub_r = NR * 0.24
    _add_central_spine(
        shell,
        model,
        r,
        fan_x=fan_x,
        hub_r=hub_r,
        spine_radius=NR * 0.34,
        spine_rear=front + 0.92 * EL,
    )
    # Outlet guide vanes / rear frame tying the core body to the nacelle wall.
    _radial_struts(
        shell,
        r,
        x_pos=fan_x + 0.34 * EL,
        inner_r=NR * 0.30,
        outer_r=_shell_radius_at(r, 0.55, inner=True),
        count=6,
        name="ogv",
    )
    _build_front_rotor(
        model,
        r,
        part_name="fan_rotor",
        joint_name="fan_spin",
        fan_x=fan_x,
        fan_r=NR * 0.76,
        hub_r=hub_r,
        hub_style="spinner",
        effort=120.0,
        velocity=120.0,
        blade_count=r.fan_blade_count,
    )


def _build_turbojet_module(model: ArticulatedObject, r: ResolvedJetEngineConfig) -> None:
    EL = r.engine_length
    NRc = r.nacelle_radius * 0.78
    front = -EL * 0.5
    shell = model.part("outer_shell")
    shell.visual(
        _build_shell_lathe(model, r, name="jet_casing_shell"),
        origin=Origin(xyz=(0.0, 0.0, r.axis_z)),
        material="nacelle",
        name="nacelle_shell",
    )

    comp_x = front + 0.14 * EL  # compressor face just inside the casing intake
    hub_r = NRc * 0.30
    _add_central_spine(
        shell,
        model,
        r,
        fan_x=comp_x,
        hub_r=hub_r,
        spine_radius=NRc * 0.40,
        spine_rear=front + 0.94 * EL,
    )
    # Front strut spider (S6 278c04): support vanes carry the rotor bearing.
    _radial_struts(
        shell,
        r,
        x_pos=comp_x + 0.20 * EL,
        inner_r=NRc * 0.30,
        outer_r=_shell_radius_at(r, 0.45, inner=True),
        count=4,
        name="front_strut",
    )
    # Flat compressor face (no spinner cone). Compressor disks have many blades.
    _build_front_rotor(
        model,
        r,
        part_name="compressor_rotor",
        joint_name="compressor_spin",
        fan_x=comp_x,
        fan_r=NRc * 0.78,
        hub_r=hub_r,
        hub_style="flat",
        effort=25.0,
        velocity=70.0,
        blade_count=max(12, r.fan_blade_count + 4),
    )


def _build_cutaway_twospool(model: ArticulatedObject, r: ResolvedJetEngineConfig) -> None:
    EL, NR = r.engine_length, r.nacelle_radius
    front = -EL * 0.5
    az = r.axis_z
    shell = model.part("outer_shell")
    # Short front cowl over the fan; the core spool is left exposed in the cage.
    shell.visual(
        _build_shell_lathe(model, r, name="jet_cutaway_cowl", length_frac=0.46),
        origin=Origin(xyz=(0.0, 0.0, az)),
        material="nacelle",
        name="nacelle_shell",
    )

    fan_x = front + 0.18 * EL
    hub_r = NR * 0.24
    tube_r = NR * 0.06
    # Long thin static bearing tube down the whole engine. Both spinning spools
    # surround it (their hubs hide it), and the OGV ties it to the front cowl, so
    # the static structure stays a single connected island while the core is left
    # visually exposed (the cutaway identity).
    shell.visual(
        Cylinder(radius=tube_r, length=EL * 0.94),
        origin=Origin(xyz=(front + 0.49 * EL, 0.0, az), rpy=_cyl_x()),
        material="dark",
        name="bearing_hub",
    )
    # Outlet guide vanes sit between the fan and the first compressor stage,
    # carrying the tube out to the cowl wall.
    _radial_struts(
        shell,
        r,
        x_pos=fan_x + 0.16 * EL,
        inner_r=tube_r,
        outer_r=NR * 0.85,
        count=6,
        name="ogv",
    )

    # Low spool: spinning fan captured on the bearing tube front.
    _build_front_rotor(
        model,
        r,
        part_name="fan_rotor",
        joint_name="fan_spin",
        fan_x=fan_x,
        fan_r=NR * 0.78,
        hub_r=hub_r,
        hub_style="spinner",
        effort=120.0,
        velocity=120.0,
        blade_count=r.fan_blade_count,
    )

    # High spool: compressor + turbine stage disks on a second CONTINUOUS axis,
    # exposed between the cowl and the tail (M3 / M4 jointless multiplicity).
    spool = model.part("core_spool")
    spool_front = front + 0.40 * EL
    spool.visual(
        Cylinder(radius=NR * 0.085, length=EL * 0.48),
        origin=Origin(xyz=(EL * 0.24, 0.0, 0.0), rpy=_cyl_x()),
        material="dark",
        name="core_shaft",
    )
    stage_hub_r = NR * 0.11

    def _stage(idx: int, kind: str, sx: float, outer_r: float, blades: int, mat: str) -> None:
        # Hub disk grips the shaft; the blade row fans out from it.
        spool.visual(
            Cylinder(radius=stage_hub_r, length=EL * 0.03),
            origin=Origin(xyz=(sx, 0.0, 0.0), rpy=_cyl_x()),
            material=mat,
            name=f"{kind}_hub_{idx}",
        )
        spool.visual(
            _blade_disk(
                model,
                hub_r=stage_hub_r,
                outer_r=outer_r,
                blade_count=blades,
                name=f"{kind}_stage_{idx}",
                chord_frac=0.5,
            ),
            origin=Origin(xyz=(sx, 0.0, 0.0)),
            material=mat,
            name=f"{kind}_stage_{idx}",
        )

    n_c = r.compressor_stage_count
    for i in range(n_c):
        frac = (i + 0.5) / n_c
        _stage(
            i, "compressor", EL * 0.03 + EL * 0.26 * frac, NR * (0.34 - 0.10 * frac), 16, "metal"
        )
    n_t = r.turbine_stage_count
    for i in range(n_t):
        frac = (i + 0.5) / n_t
        _stage(i, "turbine", EL * 0.34 + EL * 0.10 * frac, NR * (0.30 - 0.06 * frac), 14, "hot")
    model.articulation(
        "core_spin",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=spool,
        origin=Origin(xyz=(spool_front, 0.0, az)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=260.0),
    )


# --------------------------------------------------------------------------- #
# Slot A — mount_platform
# --------------------------------------------------------------------------- #


def _build_mount_platform(model: ArticulatedObject, r: ResolvedJetEngineConfig) -> None:
    shell = model.get_part("outer_shell")
    EL, NR = r.engine_length, r.nacelle_radius
    sor = _shell_outer_r(r)
    if r.mount_platform == "bare_inline":
        return

    if r.mount_platform == "top_pylon_mount":
        # Pylon hangs the engine under a wing. The engine_core local origin is
        # the crown (axis_z = -sor), so the shell top is at local z=0.
        crown_pad_x = -EL * 0.06
        # Pad straddles the crown (local z=0) so the FIXED joint origin lands on
        # real geometry and the pylon foot overlaps it (visible clamped support).
        shell.visual(
            Box((EL * 0.34, NR * 0.26, NR * 0.10)),
            origin=Origin(xyz=(crown_pad_x, 0.0, -NR * 0.03)),
            material="metal",
            name="crown_mount_pad",
        )
        pylon = model.part("pylon_mount")
        pylon.visual(
            Box((EL * 0.30, NR * 0.22, NR * 0.09)),
            origin=Origin(xyz=(crown_pad_x, 0.0, NR * 0.015)),
            material="metal",
            name="pylon_foot",
        )
        # Strut overlaps the foot below and the wing pad above (one island).
        pylon.visual(
            Box((EL * 0.22, NR * 0.14, NR * 0.64)),
            origin=Origin(xyz=(crown_pad_x, 0.0, NR * 0.34)),
            material="metal",
            name="pylon_strut",
        )
        pylon.visual(
            Box((EL * 0.50, NR * 0.60, NR * 0.12)),
            origin=Origin(xyz=(crown_pad_x, 0.0, NR * 0.66)),
            material="dark",
            name="pylon_wing_pad",
        )
        model.articulation(
            "pylon_to_engine",
            ArticulationType.FIXED,
            parent=pylon,
            child=shell,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        return

    # ground_display_stand: a separate grounded part cradles the belly (S19/S3).
    # axis_z = +sor, so the shell belly plane is local z=0 and the centreline
    # rides ``stand_height`` above the floor. A belly keel runs along the
    # underside through x=0 (touching the cowl belly — for the short cutaway it
    # reaches forward onto the cowl) so the FIXED joint anchor at the shell's
    # local origin lands on real geometry, and the saddle grips that keel at x=0.
    belly_z = r.stand_height
    keel_z = r.axis_z - sor  # local belly plane
    shell.visual(
        Box((EL * 0.58, NR * 0.5, NR * 0.06)),
        origin=Origin(xyz=(-0.15 * EL, 0.0, keel_z)),
        material="dark",
        name="belly_mount_pad",
    )
    stand = model.part("display_stand")
    stand.visual(
        Box((EL * 1.02, sor * 2.1, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="dark",
        name="base_plate",
    )
    saddle_h = NR * 0.16
    for idx, sx in enumerate((-0.18 * EL, 0.06 * EL)):
        col_top = belly_z - saddle_h * 0.5
        stand.visual(
            Box((NR * 0.22, sor * 0.55, col_top - 0.04)),
            origin=Origin(xyz=(sx, 0.0, 0.5 * (col_top + 0.04))),
            material="metal",
            name=f"stand_column_{idx}",
        )
    stand.visual(
        Box((EL * 0.46, sor * 1.05, saddle_h)),
        origin=Origin(xyz=(0.0, 0.0, belly_z - saddle_h * 0.5)),
        material="metal",
        name="stand_saddle",
    )
    model.articulation(
        "stand_to_engine",
        ArticulationType.FIXED,
        parent=stand,
        child=shell,
        origin=Origin(xyz=(0.0, 0.0, belly_z)),
    )


# --------------------------------------------------------------------------- #
# Slot C — aft_nozzle
# --------------------------------------------------------------------------- #


def _build_aft_nozzle(model: ArticulatedObject, r: ResolvedJetEngineConfig) -> None:
    shell = model.get_part("outer_shell")
    EL, NR = r.engine_length, r.nacelle_radius
    az = r.axis_z
    if r.engine_core == "cutaway_twospool":
        # Aft of the open cage: the rear retaining ring is the attach plane.
        exit_x = -EL * 0.5 + 0.92 * EL
        rear_outer = NR * 0.42
        rear_inner = NR * 0.30
    else:
        exit_x = EL * 0.5
        rear_outer = _shell_radius_at(r, 1.0)
        rear_inner = _shell_radius_at(r, 1.0, inner=True)

    if r.aft_nozzle == "static_exhaust_cone":
        if r.engine_core == "cutaway_twospool":
            # On-axis tail bullet on the bearing tube, behind the exposed spool
            # (its on-axis nose overlaps the static tube → connected).
            shell.visual(
                _lathe_solid(
                    model,
                    [(0.0, 0.0), (0.30, 0.95), (0.65, 1.0), (1.0, 0.0)],
                    EL * 0.22,
                    NR * 0.22,
                    x0=-EL * 0.5 + 0.90 * EL,
                    name="jet_exhaust_cone",
                ),
                origin=Origin(xyz=(0.0, 0.0, az)),
                material="dark",
                name="exhaust_cone",
            )
            return
        # Plug/centre-body cone seated at the exit rim, tapering aft (S19 ec6fd5).
        # The base rim sits in the shell rear wall (connected).
        shell.visual(
            _lathe_solid(
                model,
                [(0.0, 1.0), (0.45, 0.80), (0.80, 0.44), (1.0, 0.0)],
                EL * 0.26,
                rear_inner + NR * 0.03,
                x0=exit_x,
                name="jet_exhaust_cone",
            ),
            origin=Origin(xyz=(0.0, 0.0, az)),
            material="dark",
            name="exhaust_cone",
        )
        return

    if r.aft_nozzle == "static_chevron_ring":
        # High-bypass sawtooth chevron ring (S15 ad0120): static teeth on the rim.
        shell.visual(
            _ring(
                model,
                inner_r=rear_inner,
                outer_r=rear_outer,
                width=EL * 0.05,
                name="jet_chevron_ring",
            ),
            origin=Origin(xyz=(exit_x, 0.0, az)),
            material="dark",
            name="chevron_ring",
        )
        n = 14
        ring_r = 0.5 * (rear_inner + rear_outer)
        chev_w = (2.0 * math.pi * ring_r / n) * 0.78
        for i in range(n):
            ang = i * math.tau / n
            shell.visual(
                Box((EL * 0.08, NR * 0.05, chev_w)),
                origin=Origin(
                    xyz=(exit_x + EL * 0.045, math.cos(ang) * ring_r, az + math.sin(ang) * ring_r),
                    rpy=(ang, 0.0, 0.0),
                ),
                material="dark",
                name=f"chevron_{i}",
            )
        return

    # variable_petal_nozzle: a static ring + N hinged petal parts (S18 e2123c /
    # S8 4f9be8). Each petal hinges on its own REVOLUTE; ``mimic`` gangs the
    # followers to petal_00.
    n = r.petal_count
    hinge_r = 0.5 * (rear_inner + rear_outer)
    shell.visual(
        _ring(
            model, inner_r=rear_inner, outer_r=rear_outer, width=EL * 0.05, name="jet_nozzle_ring"
        ),
        origin=Origin(xyz=(exit_x, 0.0, az)),
        material="dark",
        name="nozzle_ring",
    )
    petal_len = EL * 0.18
    petal_w = (2.0 * math.pi * hinge_r / n) * 0.78
    for i in range(n):
        ang = i * math.tau / n
        # Hinge boss on the ring: gives the joint origin real parent geometry
        # and a visible support knuckle the petal seats on.
        shell.visual(
            Box((EL * 0.05, NR * 0.06, petal_w * 0.7)),
            origin=Origin(
                xyz=(exit_x, math.cos(ang) * hinge_r, az + math.sin(ang) * hinge_r),
                rpy=(ang, 0.0, 0.0),
            ),
            material="metal",
            name=f"petal_mount_{i:02d}",
        )
        petal = model.part(f"nozzle_petal_{i:02d}")
        # Lofted tapered concave petal, starting at the hinge (local x=0) so it
        # overlaps the hinge barrel — one connected petal island.
        petal.visual(
            _petal_mesh(
                model,
                length=petal_len,
                w_root=petal_w,
                w_tip=petal_w * 0.6,
                thickness=NR * 0.05,
                name=f"nozzle_petal_{i:02d}_skin",
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="metal",
            name="petal_skin",
        )
        petal.visual(
            Cylinder(radius=NR * 0.05, length=petal_w * 0.9),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="dark",
            name="hinge_barrel",
        )
        mimic = Mimic(joint="petal_00_hinge") if (r.petal_gang_mode == "mimic" and i > 0) else None
        model.articulation(
            f"petal_{i:02d}_hinge",
            ArticulationType.REVOLUTE,
            parent=shell,
            child=petal,
            origin=Origin(
                xyz=(exit_x, math.cos(ang) * hinge_r, az + math.sin(ang) * hinge_r),
                rpy=(ang, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-0.10, upper=0.42),
            mimic=mimic,
        )


# --------------------------------------------------------------------------- #
# Slot D — access_door
# --------------------------------------------------------------------------- #


def _build_access_door(model: ArticulatedObject, r: ResolvedJetEngineConfig) -> None:
    if r.access_door == "none":
        return
    shell = model.get_part("outer_shell")
    EL, NR = r.engine_length, r.nacelle_radius
    az = r.axis_z
    front = -EL * 0.5
    door_x = front + 0.34 * EL
    skin_r = _shell_radius_at_x(r, door_x)
    panel_l = EL * 0.22
    panel_w = NR * 0.5

    if r.access_door == "belly_fold_down_panel":
        # Belly hinge line runs along Y; the panel folds down (S7 3ad5d74).
        hinge_z = az - skin_r
        # Hinge hardware on the shell belly (visible support + on-origin geom).
        shell.visual(
            Cylinder(radius=NR * 0.045, length=panel_w * 1.05),
            origin=Origin(xyz=(door_x, 0.0, hinge_z), rpy=_cyl_y()),
            material="dark",
            name="belly_hinge_pin",
        )
        for sy in (-panel_w * 0.5, panel_w * 0.5):
            shell.visual(
                Box((NR * 0.07, NR * 0.10, NR * 0.10)),
                origin=Origin(xyz=(door_x, sy, hinge_z + NR * 0.04)),
                material="dark",
                name=f"belly_hinge_bracket_{0 if sy < 0 else 1}",
            )
        door = model.part("access_panel")
        door.visual(
            Box((panel_l, panel_w, NR * 0.04)),
            origin=Origin(xyz=(panel_l * 0.5, 0.0, NR * 0.02)),
            material="accent",
            name="panel_skin",
        )
        door.visual(
            Cylinder(radius=NR * 0.04, length=panel_w),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_cyl_y()),
            material="dark",
            name="moving_hinge",
        )
        door.visual(
            Box((NR * 0.08, panel_w * 0.5, NR * 0.05)),
            origin=Origin(xyz=(panel_l * 0.95, 0.0, NR * 0.02)),
            material="dark",
            name="panel_latch",
        )
        model.articulation(
            "belly_panel_hinge",
            ArticulationType.REVOLUTE,
            parent=shell,
            child=door,
            origin=Origin(xyz=(door_x, 0.0, hinge_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=1.0, lower=0.0, upper=1.30),
        )
        return

    # side_equipment_bay_door: hinge line on the +Y skin, runs along X (S18).
    hinge_y = skin_r
    shell.visual(
        Cylinder(radius=NR * 0.045, length=panel_l * 1.05),
        origin=Origin(xyz=(door_x, hinge_y, az), rpy=_cyl_x()),
        material="dark",
        name="bay_hinge_pin",
    )
    shell.visual(
        Box((panel_l * 0.9, NR * 0.10, NR * 0.42)),
        origin=Origin(xyz=(door_x, hinge_y - NR * 0.04, az)),
        material="dark",
        name="bay_housing",
    )
    shell.visual(
        Box((panel_l * 0.5, NR * 0.14, NR * 0.20)),
        origin=Origin(xyz=(door_x, hinge_y - NR * 0.06, az)),
        material="metal",
        name="bay_equipment",
    )
    door = model.part("bay_door")
    door.visual(
        Box((panel_l, NR * 0.04, panel_w)),
        origin=Origin(xyz=(panel_l * 0.5, NR * 0.02, 0.0)),
        material="accent",
        name="panel_skin",
    )
    door.visual(
        Cylinder(radius=NR * 0.04, length=panel_l),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_cyl_x()),
        material="dark",
        name="moving_hinge",
    )
    door.visual(
        Box((panel_l * 0.5, NR * 0.05, NR * 0.10)),
        origin=Origin(xyz=(panel_l * 0.95, NR * 0.02, 0.0)),
        material="dark",
        name="panel_latch",
    )
    model.articulation(
        "side_bay_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=door,
        origin=Origin(xyz=(door_x, hinge_y, az)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.2, lower=0.0, upper=1.20),
    )


# --------------------------------------------------------------------------- #
# Top-level build
# --------------------------------------------------------------------------- #

_ENGINE_CORE_BUILDERS = {
    "smooth_turbofan": _build_smooth_turbofan,
    "turbojet_module": _build_turbojet_module,
    "cutaway_twospool": _build_cutaway_twospool,
}


def build_jet_engine(
    config: JetEngineConfig | None = None, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    r = resolve_config(config or JetEngineConfig())
    model = ArticulatedObject(name="jet_engine", assets=assets)
    for name, rgba in r.palette.items():
        model.material(name, rgba=rgba)
    _ENGINE_CORE_BUILDERS[r.engine_core](model, r)
    _build_mount_platform(model, r)
    _build_aft_nozzle(model, r)
    _build_access_door(model, r)
    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_jet_engine(seed: int, *, assets: AssetContext | None = None) -> ArticulatedObject:
    return build_jet_engine(config_from_seed(seed), assets=assets)


def slot_choices_for_config(r: ResolvedJetEngineConfig) -> list[tuple[str, str]]:
    petal = (
        f"{r.petal_count}_petal_{r.petal_gang_mode}"
        if r.aft_nozzle == "variable_petal_nozzle"
        else "no_petal"
    )
    return [
        ("mount_platform", r.mount_platform),
        ("engine_core", r.engine_core),
        ("aft_nozzle", r.aft_nozzle),
        ("access_door", r.access_door),
        ("petal_multiplicity", petal),
    ]


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #


def run_jet_engine_tests(object_model: ArticulatedObject, config: JetEngineConfig) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    names = {p.name for p in object_model.parts}
    joints = {j.name: j for j in object_model.articulations}
    shell = object_model.get_part("outer_shell")

    ctx.check("outer_shell_present", "outer_shell" in names)

    # --- Category identity: at least one CONTINUOUS rotor about engine axis X ---
    primary_spin = "compressor_spin" if r.engine_core == "turbojet_module" else "fan_spin"
    rotor_name = "compressor_rotor" if r.engine_core == "turbojet_module" else "fan_rotor"
    ctx.check("primary_rotor_present", primary_spin in joints and rotor_name in names)
    if primary_spin in joints:
        j = joints[primary_spin]
        ctx.check("primary_rotor_continuous", j.articulation_type == ArticulationType.CONTINUOUS)
        ctx.check("primary_rotor_axis_x", tuple(j.axis) == (1.0, 0.0, 0.0))

    if r.engine_core == "cutaway_twospool":
        ctx.check("core_spool_present", "core_spool" in names and "core_spin" in joints)
        if "core_spin" in joints:
            jc = joints["core_spin"]
            ctx.check("core_spin_continuous", jc.articulation_type == ArticulationType.CONTINUOUS)
            ctx.check("core_spin_axis_x", tuple(jc.axis) == (1.0, 0.0, 0.0))
            ctx.check("two_independent_spools", jc.name != primary_spin and "fan_spin" in joints)
    else:
        ctx.check("single_spool_no_core_spin", "core_spin" not in joints)

    shell_visuals = {v.name for v in shell.visuals}

    def allow_against(child_part, parent_elems, reason: str) -> None:
        """Element-scoped allow for every (parent_elem, child_visual) pair that
        actually exists — the captured-interface overlaps are local & justified."""
        child_visuals = [v.name for v in child_part.visuals]
        for pe in parent_elems:
            if pe not in shell_visuals:
                continue
            for ce in child_visuals:
                ctx.allow_overlap(shell, child_part, elem_a=pe, elem_b=ce, reason=reason)

    def max_radial(aabb) -> float:
        if aabb is None:
            return -1.0
        (_xmn, ymn, zmn), (_xmx, ymx, zmx) = aabb
        return max(math.hypot(y, z - r.axis_z) for y in (ymn, ymx) for z in (zmn, zmx))

    # Central static elements the rotors are captured against / spin inside
    # (bearing tube, core body, OGV / strut spider). The spinning rotor hubs and
    # shaft surround these on the axis — a local, justified captured overlap.
    _central_prefixes = ("bearing_hub", "core_body", "ogv", "front_strut")
    central_elems = [v for v in shell_visuals if v.startswith(_central_prefixes)]

    # --- Captured rotor: element-scoped overlap + contact + concentric spin ---
    if rotor_name in names:
        rotor = object_model.get_part(rotor_name)
        allow_against(
            rotor,
            central_elems,
            "Rotor hub/shaft is captured on the static centre bearing/core spine.",
        )
        ctx.expect_contact(
            shell,
            rotor,
            elem_a="bearing_hub",
            elem_b="fan_hub",
            contact_tol=0.02,
            name="rotor_hub_seats_in_bearing",
        )
        rest = ctx.part_world_position(rotor)
        with ctx.pose({joints[primary_spin]: math.pi / 3.0}):
            turned = ctx.part_world_position(rotor)
        ctx.check(
            "rotor_spins_about_fixed_centreline",
            rest is not None
            and turned is not None
            and all(abs(a - b) < 1e-6 for a, b in zip(rest, turned)),
            details=f"rest={rest} turned={turned}",
        )

    if r.engine_core == "cutaway_twospool" and "core_spool" in names:
        spool = object_model.get_part("core_spool")
        allow_against(
            spool,
            central_elems,
            "Core spool spins around the static bearing tube inside the open cage.",
        )
        ctx.expect_within(
            spool,
            shell,
            axes="yz",
            inner_elem="core_shaft",
            outer_elem="bearing_hub",
            margin=0.08,
            name="core_shaft_coaxial_with_bearing",
        )

    # --- Aft nozzle ---
    if r.aft_nozzle == "variable_petal_nozzle":
        petal_parts = {f"nozzle_petal_{i:02d}" for i in range(r.petal_count)}
        ctx.check(
            "petal_part_count",
            petal_parts.issubset(names),
            details=f"missing={sorted(petal_parts - names)}",
        )
        petal_joints = [joints.get(f"petal_{i:02d}_hinge") for i in range(r.petal_count)]
        ctx.check("petal_joint_count", all(j is not None for j in petal_joints))
        ctx.check(
            "petals_revolute",
            all(j.articulation_type == ArticulationType.REVOLUTE for j in petal_joints if j),
        )
        if r.petal_gang_mode == "mimic":
            ctx.check(
                "petal_followers_mimic",
                all(getattr(j, "mimic", None) is not None for j in petal_joints[1:] if j),
            )
        for i in range(r.petal_count):
            pname = f"nozzle_petal_{i:02d}"
            if pname in names:
                allow_against(
                    object_model.get_part(pname),
                    [f"petal_mount_{i:02d}", "nozzle_ring", "nacelle_shell"],
                    "Petal hinge barrel is captured on the static nozzle mount ring.",
                )
        # Petal seats on its hinge boss and opens radially outward (posed AABB).
        p0 = object_model.get_part("nozzle_petal_00")
        ctx.expect_contact(
            shell,
            p0,
            elem_a="petal_mount_00",
            elem_b="hinge_barrel",
            contact_tol=0.03,
            name="petal00_seats_on_mount",
        )
        j0 = joints["petal_00_hinge"]
        with ctx.pose({j0: -0.08}):
            r_closed = max_radial(ctx.part_element_world_aabb(p0, elem="petal_skin"))
        with ctx.pose({j0: 0.40}):
            r_open = max_radial(ctx.part_element_world_aabb(p0, elem="petal_skin"))
        ctx.check(
            "petals_open_outward",
            r_open > r_closed + 0.01,
            details=f"closed_r={r_closed:.4f} open_r={r_open:.4f}",
        )
    else:
        ctx.check("no_petals", not any(n.startswith("nozzle_petal_") for n in names))

    # --- Access door ---
    if r.access_door == "belly_fold_down_panel":
        ctx.check("belly_panel_present", "access_panel" in names and "belly_panel_hinge" in joints)
        if "access_panel" in names:
            door = object_model.get_part("access_panel")
            allow_against(
                door,
                [
                    "belly_hinge_pin",
                    "belly_hinge_bracket_0",
                    "belly_hinge_bracket_1",
                    "nacelle_shell",
                    *central_elems,
                ],
                "Belly panel hinge/skin sits flush over the nacelle belly frame.",
            )
            ctx.expect_contact(
                shell,
                door,
                elem_a="belly_hinge_pin",
                elem_b="moving_hinge",
                contact_tol=0.03,
                name="belly_panel_hinge_seated",
            )
            jd = joints["belly_panel_hinge"]
            with ctx.pose({jd: 0.0}):
                closed = ctx.part_element_world_aabb(door, elem="panel_skin")
            with ctx.pose({jd: 1.20}):
                opened = ctx.part_element_world_aabb(door, elem="panel_skin")
            ctx.check(
                "belly_panel_folds_down",
                closed is not None and opened is not None and opened[0][2] < closed[0][2] - 0.05,
                details=f"closed={closed} open={opened}",
            )
    elif r.access_door == "side_equipment_bay_door":
        ctx.check("bay_door_present", "bay_door" in names and "side_bay_hinge" in joints)
        if "bay_door" in names:
            door = object_model.get_part("bay_door")
            allow_against(
                door,
                ["bay_hinge_pin", "bay_housing", "bay_equipment", "nacelle_shell", *central_elems],
                "Bay door hinge/skin sits flush over the equipment bay frame.",
            )
            ctx.expect_contact(
                shell,
                door,
                elem_a="bay_hinge_pin",
                elem_b="moving_hinge",
                contact_tol=0.03,
                name="bay_door_hinge_seated",
            )
            jd = joints["side_bay_hinge"]
            with ctx.pose({jd: 0.0}):
                closed = ctx.part_element_world_aabb(door, elem="panel_skin")
            with ctx.pose({jd: 1.10}):
                opened = ctx.part_element_world_aabb(door, elem="panel_skin")
            ctx.check(
                "bay_door_swings_out",
                closed is not None and opened is not None and opened[1][1] > closed[1][1] + 0.05,
                details=f"closed={closed} open={opened}",
            )

    # --- Mount platform ---
    if r.mount_platform == "ground_display_stand":
        ctx.check("stand_present", "display_stand" in names and "stand_to_engine" in joints)
        if "display_stand" in names:
            stand = object_model.get_part("display_stand")
            for se in ("stand_saddle",):
                for ce in ("belly_mount_pad", "nacelle_shell"):
                    ctx.allow_overlap(
                        stand,
                        shell,
                        elem_a=se,
                        elem_b=ce,
                        reason="Engine belly is cradled in the stand saddle.",
                    )
            ctx.expect_contact(
                stand,
                shell,
                elem_a="stand_saddle",
                elem_b="belly_mount_pad",
                contact_tol=0.04,
                name="engine_seated_on_saddle",
            )
    elif r.mount_platform == "top_pylon_mount":
        ctx.check("pylon_present", "pylon_mount" in names and "pylon_to_engine" in joints)
        if "pylon_mount" in names:
            pylon = object_model.get_part("pylon_mount")
            for ce in ("crown_mount_pad", "nacelle_shell"):
                ctx.allow_overlap(
                    pylon,
                    shell,
                    elem_a="pylon_foot",
                    elem_b=ce,
                    reason="Pylon foot clamps onto the nacelle crown.",
                )
            ctx.expect_contact(
                pylon,
                shell,
                elem_a="pylon_foot",
                elem_b="crown_mount_pad",
                contact_tol=0.04,
                name="pylon_clamps_crown",
            )

    # --- Default baseline-style QC stack (also enforced by the compiler) ---
    ctx.check_model_valid()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    return ctx.report()


__all__ = [
    "JetEngineConfig",
    "ResolvedJetEngineConfig",
    "config_from_seed",
    "resolve_config",
    "build_jet_engine",
    "build_seeded_jet_engine",
    "slot_choices_for_seed",
    "run_jet_engine_tests",
]
