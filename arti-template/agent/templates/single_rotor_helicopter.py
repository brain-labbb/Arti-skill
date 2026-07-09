"""Single rotor helicopter modular procedural template.

Slots:

* ``airframe_landing``: skid utility or wheeled transport fuselage.
* ``main_rotor``: variable-blade rotor family.
* ``tail_rotor``: compact or fin-mounted anti-torque rotor with variable blade count.
* ``doors_service``: hinged service/crew doors, cockpit plus sliding cabin
  door, or no articulated doors.
* ``mission_profile``: non-articulated mission equipment and silhouette
  details that make seeded helicopters visibly distinct.

seed=0 anchors to the fire utility skid helicopter family.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Literal

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

__modular__ = True

AirframeLandingModule = Literal["fire_utility_skid_airframe", "offshore_wheeled_airframe"]
MainRotorModule = Literal["five_blade_fire_rotor", "tall_mast_transport_rotor"]
TailRotorModule = Literal["compact_tail_rotor", "fin_mounted_tail_rotor"]
DoorsServiceModule = Literal["service_and_crew_hinges", "cockpit_hinges_and_cabin_slide", "none"]
MissionProfile = Literal[
    "fire_utility",
    "rescue_hoist",
    "agricultural_sprayer",
    "tour_bubble",
    "police_observer",
    "offshore_float_pack",
]
PaletteTheme = Literal[
    "fire_rescue",
    "offshore_orange",
    "navy_gray",
    "medevac_white",
    "agricultural_green",
    "tour_blue",
]

AIRFRAME_MODULES: tuple[AirframeLandingModule, ...] = (
    "fire_utility_skid_airframe",
    "offshore_wheeled_airframe",
)
MAIN_ROTOR_MODULES: tuple[MainRotorModule, ...] = (
    "five_blade_fire_rotor",
    "tall_mast_transport_rotor",
)
TAIL_ROTOR_MODULES: tuple[TailRotorModule, ...] = (
    "compact_tail_rotor",
    "fin_mounted_tail_rotor",
)
DOOR_MODULES: tuple[DoorsServiceModule, ...] = (
    "service_and_crew_hinges",
    "cockpit_hinges_and_cabin_slide",
    "none",
)
MISSION_PROFILES: tuple[MissionProfile, ...] = (
    "fire_utility",
    "rescue_hoist",
    "agricultural_sprayer",
    "tour_bubble",
    "police_observer",
    "offshore_float_pack",
)

PALETTES: dict[PaletteTheme, dict[str, tuple[float, float, float, float]]] = {
    "fire_rescue": {
        "body": (0.78, 0.08, 0.06, 1.0),
        "accent": (0.93, 0.92, 0.84, 1.0),
        "dark": (0.08, 0.09, 0.10, 1.0),
        "metal": (0.58, 0.60, 0.63, 1.0),
        "rotor": (0.18, 0.19, 0.20, 1.0),
        "glass": (0.10, 0.22, 0.34, 0.70),
        "tire": (0.03, 0.03, 0.035, 1.0),
        "warning": (0.95, 0.58, 0.08, 1.0),
    },
    "offshore_orange": {
        "body": (0.92, 0.88, 0.78, 1.0),
        "accent": (0.92, 0.38, 0.06, 1.0),
        "dark": (0.10, 0.11, 0.12, 1.0),
        "metal": (0.60, 0.62, 0.64, 1.0),
        "rotor": (0.20, 0.21, 0.22, 1.0),
        "glass": (0.08, 0.20, 0.30, 0.70),
        "tire": (0.025, 0.025, 0.03, 1.0),
        "warning": (0.98, 0.68, 0.10, 1.0),
    },
    "navy_gray": {
        "body": (0.42, 0.46, 0.49, 1.0),
        "accent": (0.68, 0.70, 0.68, 1.0),
        "dark": (0.07, 0.08, 0.09, 1.0),
        "metal": (0.54, 0.56, 0.58, 1.0),
        "rotor": (0.15, 0.16, 0.17, 1.0),
        "glass": (0.08, 0.18, 0.28, 0.70),
        "tire": (0.025, 0.025, 0.03, 1.0),
        "warning": (0.86, 0.20, 0.10, 1.0),
    },
    "medevac_white": {
        "body": (0.88, 0.90, 0.86, 1.0),
        "accent": (0.84, 0.08, 0.08, 1.0),
        "dark": (0.08, 0.10, 0.11, 1.0),
        "metal": (0.56, 0.58, 0.60, 1.0),
        "rotor": (0.16, 0.17, 0.18, 1.0),
        "glass": (0.10, 0.24, 0.34, 0.70),
        "tire": (0.025, 0.025, 0.03, 1.0),
        "warning": (0.96, 0.66, 0.08, 1.0),
    },
    "agricultural_green": {
        "body": (0.16, 0.42, 0.22, 1.0),
        "accent": (0.93, 0.88, 0.22, 1.0),
        "dark": (0.07, 0.09, 0.07, 1.0),
        "metal": (0.56, 0.58, 0.54, 1.0),
        "rotor": (0.14, 0.16, 0.14, 1.0),
        "glass": (0.10, 0.24, 0.26, 0.70),
        "tire": (0.025, 0.025, 0.025, 1.0),
        "warning": (0.98, 0.55, 0.08, 1.0),
    },
    "tour_blue": {
        "body": (0.10, 0.36, 0.62, 1.0),
        "accent": (0.92, 0.92, 0.82, 1.0),
        "dark": (0.06, 0.08, 0.12, 1.0),
        "metal": (0.56, 0.58, 0.62, 1.0),
        "rotor": (0.12, 0.14, 0.18, 1.0),
        "glass": (0.08, 0.28, 0.42, 0.72),
        "tire": (0.025, 0.025, 0.03, 1.0),
        "warning": (0.96, 0.72, 0.10, 1.0),
    },
}


@dataclass(frozen=True)
class SingleRotorHelicopterConfig:
    airframe_landing_module: AirframeLandingModule | None = None
    main_rotor_module: MainRotorModule | None = None
    tail_rotor_module: TailRotorModule | None = None
    doors_service_module: DoorsServiceModule | None = None
    mission_profile: MissionProfile = "fire_utility"
    palette_theme: PaletteTheme = "fire_rescue"
    fuselage_length: float = 4.3
    fuselage_width: float = 1.25
    fuselage_height: float = 1.05
    tail_length: float = 3.3
    main_rotor_radius: float = 3.15
    tail_rotor_radius: float = 0.58
    main_blade_count: int = 5
    tail_blade_count: int = 4
    palette: dict[str, tuple[float, float, float, float]] = field(
        default_factory=lambda: dict(PALETTES["fire_rescue"])
    )


@dataclass(frozen=True)
class ResolvedSingleRotorHelicopterConfig:
    airframe_landing_module: AirframeLandingModule
    main_rotor_module: MainRotorModule
    tail_rotor_module: TailRotorModule
    doors_service_module: DoorsServiceModule
    mission_profile: MissionProfile
    palette_theme: PaletteTheme
    fuselage_length: float
    fuselage_width: float
    fuselage_height: float
    tail_length: float
    main_rotor_radius: float
    tail_rotor_radius: float
    main_blade_count: int
    tail_blade_count: int
    mast_origin: tuple[float, float, float]
    tail_rotor_origin: tuple[float, float, float]
    palette: dict[str, tuple[float, float, float, float]]


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _cyl_x() -> tuple[float, float, float]:
    return (0.0, math.pi / 2.0, 0.0)


def _cyl_y() -> tuple[float, float, float]:
    return (math.pi / 2.0, 0.0, 0.0)


def _mesh(model: ArticulatedObject, geometry: MeshGeometry, name: str):
    if model.assets is not None:
        return mesh_from_geometry(geometry, model.assets.mesh_path(f"{name}.obj"))
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material: str, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_tube_mesh(
    part,
    model: ArticulatedObject,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    material: str,
    name: str,
    samples_per_segment: int = 10,
    radial_segments: int = 14,
) -> None:
    part.visual(
        _mesh(
            model,
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=samples_per_segment,
                radial_segments=radial_segments,
                cap_ends=True,
            ),
            name,
        ),
        material=material,
        name=name,
    )


def _yz_section(
    station_x: float,
    width: float,
    height: float,
    *,
    center_z: float,
    corner: float,
) -> list[tuple[float, float, float]]:
    return [
        (station_x, y, z + center_z)
        for y, z in rounded_rect_profile(width, height, corner, corner_segments=8)
    ]


def _fuselage_shell_geometry(r: ResolvedSingleRotorHelicopterConfig) -> MeshGeometry:
    L, W, H, T = r.fuselage_length, r.fuselage_width, r.fuselage_height, r.tail_length
    if r.airframe_landing_module == "offshore_wheeled_airframe":
        sections = [
            _yz_section(L * 0.72, W * 0.34, H * 0.34, center_z=H * 0.58, corner=W * 0.040),
            _yz_section(L * 0.52, W * 0.90, H * 0.78, center_z=H * 0.62, corner=W * 0.110),
            _yz_section(L * 0.24, W * 1.12, H * 1.08, center_z=H * 0.66, corner=W * 0.145),
            _yz_section(-L * 0.10, W * 1.10, H * 1.10, center_z=H * 0.66, corner=W * 0.140),
            _yz_section(-L * 0.40, W * 0.92, H * 0.82, center_z=H * 0.70, corner=W * 0.105),
            _yz_section(
                -L * 0.50 - T * 0.15, W * 0.48, H * 0.42, center_z=H * 0.86, corner=W * 0.060
            ),
            _yz_section(
                -L * 0.50 - T * 0.55, W * 0.30, H * 0.26, center_z=H * 0.92, corner=W * 0.040
            ),
            _yz_section(
                r.tail_rotor_origin[0] + 0.18,
                W * 0.22,
                H * 0.20,
                center_z=H * 0.98,
                corner=W * 0.030,
            ),
        ]
    else:
        sections = [
            _yz_section(L * 0.70, W * 0.24, H * 0.28, center_z=H * 0.58, corner=W * 0.030),
            _yz_section(L * 0.52, W * 0.72, H * 0.66, center_z=H * 0.58, corner=W * 0.095),
            _yz_section(L * 0.24, W * 1.04, H * 1.04, center_z=H * 0.62, corner=W * 0.140),
            _yz_section(-L * 0.08, W * 1.06, H * 1.08, center_z=H * 0.64, corner=W * 0.145),
            _yz_section(-L * 0.36, W * 0.82, H * 0.76, center_z=H * 0.72, corner=W * 0.100),
            _yz_section(
                -L * 0.48 - T * 0.14, W * 0.42, H * 0.38, center_z=H * 0.88, corner=W * 0.060
            ),
            _yz_section(
                -L * 0.48 - T * 0.52, W * 0.28, H * 0.24, center_z=H * 0.94, corner=W * 0.040
            ),
            _yz_section(
                r.tail_rotor_origin[0] + 0.15,
                W * 0.20,
                H * 0.18,
                center_z=H * 1.00,
                corner=W * 0.030,
            ),
        ]
    return repair_loft(section_loft(sections))


def _tapered_blade_geometry(
    length: float,
    *,
    root_chord: float,
    tip_chord: float,
    thickness: float,
    sweep: float = 0.08,
) -> MeshGeometry:
    geom = MeshGeometry()
    root_x = -length * 0.5
    tip_x = length * 0.5
    verts = [
        (root_x, -root_chord * 0.5, -thickness * 0.5),
        (root_x, root_chord * 0.5, -thickness * 0.5),
        (tip_x, -tip_chord * 0.5 + sweep, -thickness * 0.38),
        (tip_x, tip_chord * 0.5 + sweep, -thickness * 0.38),
        (root_x, -root_chord * 0.45, thickness * 0.5),
        (root_x, root_chord * 0.45, thickness * 0.5),
        (tip_x, -tip_chord * 0.42 + sweep, thickness * 0.34),
        (tip_x, tip_chord * 0.42 + sweep, thickness * 0.34),
    ]
    for vertex in verts:
        geom.add_vertex(*vertex)
    for a, b, c in (
        (0, 2, 3),
        (0, 3, 1),
        (4, 5, 7),
        (4, 7, 6),
        (0, 4, 6),
        (0, 6, 2),
        (1, 3, 7),
        (1, 7, 5),
        (0, 1, 5),
        (0, 5, 4),
        (2, 6, 7),
        (2, 7, 3),
    ):
        geom.add_face(a, b, c)
    return geom


def config_from_seed(seed: int) -> SingleRotorHelicopterConfig:
    if seed == 0:
        return SingleRotorHelicopterConfig(
            airframe_landing_module="fire_utility_skid_airframe",
            main_rotor_module="five_blade_fire_rotor",
            tail_rotor_module="compact_tail_rotor",
            doors_service_module="service_and_crew_hinges",
            mission_profile="fire_utility",
            palette_theme="fire_rescue",
            fuselage_length=4.3,
            fuselage_width=1.25,
            fuselage_height=1.05,
            tail_length=3.3,
            main_rotor_radius=3.15,
            tail_rotor_radius=0.58,
            main_blade_count=5,
            tail_blade_count=4,
        )
    rng = random.Random(seed)
    airframe: AirframeLandingModule = rng.choice(AIRFRAME_MODULES)
    main_rotor: MainRotorModule = rng.choice(MAIN_ROTOR_MODULES)
    tail_rotor: TailRotorModule = rng.choice(TAIL_ROTOR_MODULES)
    doors: DoorsServiceModule = rng.choice(DOOR_MODULES)
    profile: MissionProfile = rng.choice(MISSION_PROFILES)
    if profile == "agricultural_sprayer":
        theme: PaletteTheme = rng.choice(("agricultural_green", "offshore_orange"))
    elif profile == "rescue_hoist":
        theme = rng.choice(("medevac_white", "fire_rescue", "offshore_orange"))
    elif profile == "tour_bubble":
        theme = rng.choice(("tour_blue", "offshore_orange", "navy_gray"))
    elif profile == "police_observer":
        theme = rng.choice(("navy_gray", "tour_blue", "medevac_white"))
    elif profile == "offshore_float_pack":
        theme = rng.choice(("offshore_orange", "medevac_white", "navy_gray"))
    else:
        theme = rng.choice(("fire_rescue", "offshore_orange", "medevac_white"))
    return SingleRotorHelicopterConfig(
        airframe_landing_module=airframe,
        main_rotor_module=main_rotor,
        tail_rotor_module=tail_rotor,
        doors_service_module=doors,
        mission_profile=profile,
        palette_theme=theme,
        fuselage_length=round(rng.uniform(3.6, 5.1), 4),
        fuselage_width=round(rng.uniform(1.05, 1.65), 4),
        fuselage_height=round(rng.uniform(0.90, 1.36), 4),
        tail_length=round(rng.uniform(2.75, 4.10), 4),
        main_rotor_radius=round(rng.uniform(2.65, 3.75), 4),
        tail_rotor_radius=round(rng.uniform(0.46, 0.74), 4),
        main_blade_count=rng.randint(3, 8),
        tail_blade_count=rng.randint(2, 5),
    )


def resolve_config(config: SingleRotorHelicopterConfig) -> ResolvedSingleRotorHelicopterConfig:
    airframe = config.airframe_landing_module or "fire_utility_skid_airframe"
    main_rotor = config.main_rotor_module or "five_blade_fire_rotor"
    tail_rotor = config.tail_rotor_module or "compact_tail_rotor"
    doors = config.doors_service_module or "service_and_crew_hinges"
    profile = config.mission_profile
    if airframe not in AIRFRAME_MODULES:
        raise ValueError(f"Unsupported airframe_landing_module: {airframe}")
    if main_rotor not in MAIN_ROTOR_MODULES:
        raise ValueError(f"Unsupported main_rotor_module: {main_rotor}")
    if tail_rotor not in TAIL_ROTOR_MODULES:
        raise ValueError(f"Unsupported tail_rotor_module: {tail_rotor}")
    if doors not in DOOR_MODULES:
        raise ValueError(f"Unsupported doors_service_module: {doors}")
    if profile not in MISSION_PROFILES:
        raise ValueError(f"Unsupported mission_profile: {profile}")
    if config.palette_theme not in PALETTES:
        raise ValueError(f"Unsupported palette_theme: {config.palette_theme}")
    length = _clamp(config.fuselage_length, 3.2, 5.6)
    width = _clamp(config.fuselage_width, 0.95, 1.85)
    height = _clamp(config.fuselage_height, 0.78, 1.55)
    tail_length = _clamp(config.tail_length, 2.4, 4.5)
    mast_x = -length * 0.08 if airframe == "offshore_wheeled_airframe" else -length * 0.03
    if airframe == "offshore_wheeled_airframe":
        mast_z = height * 1.42 + 0.28
    else:
        mast_z = height * 1.35 + 0.21
    tail_x = -length * 0.5 - tail_length - 0.20
    tail_z = height * (1.46 if tail_rotor == "fin_mounted_tail_rotor" else 1.36)
    return ResolvedSingleRotorHelicopterConfig(
        airframe_landing_module=airframe,
        main_rotor_module=main_rotor,
        tail_rotor_module=tail_rotor,
        doors_service_module=doors,
        mission_profile=profile,
        palette_theme=config.palette_theme,
        fuselage_length=length,
        fuselage_width=width,
        fuselage_height=height,
        tail_length=tail_length,
        main_rotor_radius=_clamp(config.main_rotor_radius, 2.25, 4.10),
        tail_rotor_radius=_clamp(config.tail_rotor_radius, 0.38, 0.86),
        main_blade_count=max(3, min(8, int(config.main_blade_count))),
        tail_blade_count=max(2, min(5, int(config.tail_blade_count))),
        mast_origin=(mast_x, 0.0, mast_z),
        tail_rotor_origin=(tail_x, 0.0, tail_z),
        palette=dict(PALETTES[config.palette_theme]),
    )


def _add_windows(airframe, r: ResolvedSingleRotorHelicopterConfig) -> None:
    L, W, H = r.fuselage_length, r.fuselage_width, r.fuselage_height
    airframe.visual(
        Box((L * 0.24, 0.05, H * 0.34)),
        origin=Origin(xyz=(L * 0.34, 0, H * 0.83), rpy=(0.0, 0.0, 0.0)),
        material="glass",
        name="front_windshield",
    )
    airframe.visual(
        Box((L * 0.025, 0.060, H * 0.38)),
        origin=Origin(xyz=(L * 0.34, 0, H * 0.84)),
        material="dark",
        name="windshield_center_post",
    )
    airframe.visual(
        Box((L * 0.26, 0.070, H * 0.035)),
        origin=Origin(xyz=(L * 0.34, 0, H * 1.02)),
        material="dark",
        name="windshield_header",
    )
    for side_name, y in (("port", W * 0.505), ("starboard", -W * 0.505)):
        airframe.visual(
            Box((L * 0.24, 0.045, H * 0.36)),
            origin=Origin(xyz=(L * 0.18, y, H * 0.78)),
            material="glass",
            name=f"{side_name}_cockpit_window",
        )
        airframe.visual(
            Box((L * 0.26, 0.035, H * 0.040)),
            origin=Origin(xyz=(L * 0.18, y, H * 0.99)),
            material="dark",
            name=f"{side_name}_cockpit_window_header",
        )
        airframe.visual(
            Box((L * 0.26, 0.035, H * 0.040)),
            origin=Origin(xyz=(L * 0.18, y, H * 0.57)),
            material="dark",
            name=f"{side_name}_cockpit_window_sill",
        )
        airframe.visual(
            Box((L * 0.28, 0.045, H * 0.30)),
            origin=Origin(xyz=(-L * 0.15, y, H * 0.77)),
            material="glass",
            name=f"{side_name}_cabin_window",
        )
        airframe.visual(
            Box((L * 0.30, 0.035, H * 0.035)),
            origin=Origin(xyz=(-L * 0.15, y, H * 0.94)),
            material="dark",
            name=f"{side_name}_cabin_window_header",
        )
        airframe.visual(
            Box((L * 0.30, 0.035, H * 0.035)),
            origin=Origin(xyz=(-L * 0.15, y, H * 0.60)),
            material="dark",
            name=f"{side_name}_cabin_window_sill",
        )


def _add_surface_detailing(airframe, r: ResolvedSingleRotorHelicopterConfig) -> None:
    L, W, H = r.fuselage_length, r.fuselage_width, r.fuselage_height
    for side_name, y, sign in (("port", W * 0.535, 1.0), ("starboard", -W * 0.535, -1.0)):
        for idx, x in enumerate((L * 0.30, L * 0.08, -L * 0.16, -L * 0.36)):
            airframe.visual(
                Box((0.028, 0.026, H * 0.70)),
                origin=Origin(xyz=(x, y, H * 0.67)),
                material="dark",
                name=f"{side_name}_vertical_skin_seam_{idx}",
            )
        for idx, z in enumerate((H * 0.42, H * 0.98)):
            airframe.visual(
                Box((L * 0.64, 0.024, H * 0.022)),
                origin=Origin(xyz=(-L * 0.06, y, z)),
                material="dark",
                name=f"{side_name}_horizontal_skin_seam_{idx}",
            )
        for idx, x in enumerate((-L * 0.30, -L * 0.12, L * 0.06)):
            airframe.visual(
                Box((L * 0.095, 0.030, H * 0.12)),
                origin=Origin(xyz=(x, y + sign * 0.010, H * 1.07)),
                material="dark",
                name=f"{side_name}_engine_louver_{idx}",
            )
        for idx, x in enumerate((-L * 0.38, -L * 0.28, -L * 0.18, -L * 0.08, L * 0.02)):
            airframe.visual(
                Sphere(radius=0.014),
                origin=Origin(xyz=(x, y + sign * 0.012, H * 0.32)),
                material="metal",
                name=f"{side_name}_lower_rivet_{idx}",
            )
    airframe.visual(
        Box((L * 0.36, W * 0.20, H * 0.040)),
        origin=Origin(xyz=(-L * 0.16, 0, H * 1.24)),
        material="metal",
        name="roof_transmission_access_frame",
    )
    airframe.visual(
        Box((L * 0.22, W * 0.13, H * 0.032)),
        origin=Origin(xyz=(-L * 0.16, 0, H * 1.265)),
        material="dark",
        name="roof_transmission_access_lid",
    )


def _add_tail_drive_supports(
    airframe,
    r: ResolvedSingleRotorHelicopterConfig,
    *,
    cover_x: float,
    cover_z: float,
    cover_length: float,
) -> None:
    L, W, H = r.fuselage_length, r.fuselage_width, r.fuselage_height
    for idx, frac in enumerate((-0.36, 0.0, 0.36)):
        x = cover_x + cover_length * frac
        airframe.visual(
            Box((0.045, W * 0.11, H * 0.135)),
            origin=Origin(xyz=(x, 0, cover_z - H * 0.055)),
            material="metal",
            name=f"tail_drive_cover_stanchion_{idx}",
        )
        airframe.visual(
            Box((L * 0.034, W * 0.34, H * 0.040)),
            origin=Origin(xyz=(x, 0, cover_z - H * 0.140)),
            material="metal",
            name=f"tail_drive_cover_saddle_{idx}",
        )
        for side_name, y in (("port", W * 0.145), ("starboard", -W * 0.145)):
            airframe.visual(
                Box((L * 0.030, W * 0.030, H * 0.11)),
                origin=Origin(xyz=(x, y, cover_z - H * 0.090)),
                material="metal",
                name=f"{side_name}_tail_drive_cover_clamp_{idx}",
            )


def _add_mission_profile_details(airframe, r: ResolvedSingleRotorHelicopterConfig) -> None:
    L, W, H = r.fuselage_length, r.fuselage_width, r.fuselage_height
    profile = r.mission_profile
    if profile == "fire_utility":
        airframe.visual(
            Cylinder(radius=W * 0.18, length=L * 0.46),
            origin=Origin(xyz=(-L * 0.06, 0, H * 0.30), rpy=_cyl_x()),
            material="warning",
            name="belly_water_tank",
        )
        for idx, x in enumerate((-L * 0.22, L * 0.10)):
            airframe.visual(
                Box((0.050, W * 1.08, H * 0.055)),
                origin=Origin(xyz=(x, 0, H * 0.43)),
                material="dark",
                name=f"water_tank_retaining_band_{idx}",
            )
        airframe.visual(
            Cylinder(radius=0.045, length=W * 0.82),
            origin=Origin(xyz=(-L * 0.32, 0, H * 0.26), rpy=_cyl_y()),
            material="metal",
            name="tank_pickup_manifold",
        )
    elif profile == "rescue_hoist":
        side_y = W * 0.58
        airframe.visual(
            Box((0.18, 0.11, H * 0.46)),
            origin=Origin(xyz=(L * 0.02, side_y, H * 0.94)),
            material="metal",
            name="rescue_hoist_pylon",
        )
        airframe.visual(
            Box((0.48, 0.090, H * 0.070)),
            origin=Origin(xyz=(L * 0.14, side_y + W * 0.080, H * 1.14)),
            material="dark",
            name="rescue_hoist_boom",
        )
        airframe.visual(
            Box((0.24, W * 0.10, H * 0.090)),
            origin=Origin(xyz=(L * 0.02, side_y + W * 0.030, H * 1.13)),
            material="metal",
            name="rescue_hoist_roof_clamp",
        )
        airframe.visual(
            Box((0.34, 0.055, H * 0.11)),
            origin=Origin(xyz=(L * 0.05, side_y + W * 0.025, H * 1.03), rpy=(0, 0, -0.35)),
            material="metal",
            name="rescue_hoist_diagonal_brace",
        )
    elif profile == "agricultural_sprayer":
        airframe.visual(
            Cylinder(radius=W * 0.25, length=L * 0.42),
            origin=Origin(xyz=(-L * 0.10, 0, H * 1.08), rpy=_cyl_x()),
            material="accent",
            name="chemical_tank",
        )
        airframe.visual(
            Cylinder(radius=W * 0.13, length=L * 0.22),
            origin=Origin(xyz=(-L * 0.38, 0, H * 1.04), rpy=_cyl_x()),
            material="warning",
            name="tank_fill_dome",
        )
        airframe.visual(
            Box((0.070, W * 2.20, H * 0.050)),
            origin=Origin(xyz=(-L * 0.18, 0, H * 0.30)),
            material="dark",
            name="spray_boom_crossbar",
        )
        for side_name, y in (("port", W * 1.04), ("starboard", -W * 1.04)):
            airframe.visual(
                Box((0.12, 0.050, H * 0.42)),
                origin=Origin(xyz=(-L * 0.18, y, H * 0.52)),
                material="metal",
                name=f"{side_name}_spray_boom_hanger",
            )
    elif profile == "tour_bubble":
        for side_name, y in (("port", W * 0.565), ("starboard", -W * 0.565)):
            airframe.visual(
                Sphere(radius=W * 0.24),
                origin=Origin(xyz=(L * 0.16, y, H * 0.82)),
                material="glass",
                name=f"{side_name}_bubble_window",
            )
            airframe.visual(
                Box((L * 0.50, 0.032, H * 0.030)),
                origin=Origin(xyz=(-L * 0.02, y, H * 1.08)),
                material="metal",
                name=f"{side_name}_panorama_roof_rail",
            )
        airframe.visual(
            Sphere(radius=W * 0.30),
            origin=Origin(xyz=(L * 0.58, 0, H * 0.78)),
            material="glass",
            name="panoramic_nose_bubble",
        )
        airframe.visual(
            Box((L * 0.22, W * 0.64, H * 0.22)),
            origin=Origin(xyz=(-L * 0.44, 0, H * 0.44)),
            material="accent",
            name="aft_luggage_pod",
        )
    elif profile == "police_observer":
        airframe.visual(
            Cylinder(radius=0.16, length=0.18),
            origin=Origin(xyz=(L * 0.44, 0, H * 0.16), rpy=_cyl_y()),
            material="dark",
            name="nose_sensor_turret",
        )
        airframe.visual(
            Sphere(radius=0.11),
            origin=Origin(xyz=(L * 0.50, 0, H * 0.16)),
            material="glass",
            name="sensor_glass_dome",
        )
        airframe.visual(
            Cylinder(radius=0.055, length=0.42),
            origin=Origin(xyz=(L * 0.24, W * 0.70, H * 0.56), rpy=_cyl_y()),
            material="dark",
            name="side_searchlight_barrel",
        )
        airframe.visual(
            Box((0.22, 0.090, H * 0.15)),
            origin=Origin(xyz=(L * 0.24, W * 0.57, H * 0.56)),
            material="metal",
            name="searchlight_mount_bracket",
        )
        airframe.visual(
            Box((L * 0.52, 0.040, H * 0.070)),
            origin=Origin(xyz=(-L * 0.04, W * 0.57, H * 0.42)),
            material="warning",
            name="police_side_stripe",
        )
    elif profile == "offshore_float_pack":
        for side_name, y in (("port", W * 0.78), ("starboard", -W * 0.78)):
            airframe.visual(
                Cylinder(radius=H * 0.16, length=L * 0.58),
                origin=Origin(xyz=(-L * 0.04, y, H * 0.28), rpy=_cyl_x()),
                material="warning",
                name=f"{side_name}_emergency_float_bag",
            )
            for idx, x in enumerate((-L * 0.28, L * 0.18)):
                airframe.visual(
                    Box((0.080, 0.060, H * 0.32)),
                    origin=Origin(xyz=(x, y, H * 0.47)),
                    material="metal",
                    name=f"{side_name}_float_bag_strut_{idx}",
                )
        airframe.visual(
            Box((L * 0.38, W * 0.14, H * 0.18)),
            origin=Origin(xyz=(-L * 0.34, 0, H * 1.20)),
            material="dark",
            name="offshore_beacon_fairing",
        )


def _build_fire_airframe(model: ArticulatedObject, r: ResolvedSingleRotorHelicopterConfig):
    airframe = model.part("airframe")
    L, W, H, T = r.fuselage_length, r.fuselage_width, r.fuselage_height, r.tail_length
    airframe.visual(
        _mesh(model, _fuselage_shell_geometry(r), "fire_fuselage_shell"),
        material="body",
        name="fuselage_shell",
    )
    airframe.visual(
        Box((L * 0.56, W * 0.62, H * 0.58)),
        origin=Origin(xyz=(-L * 0.02, 0, H * 0.58)),
        material="body",
        name="cabin_core",
    )
    airframe.visual(
        Box((L * 0.72, W * 0.92, H * 0.16)),
        origin=Origin(xyz=(-L * 0.03, 0, H * 0.16)),
        material="dark",
        name="belly_keel",
    )
    airframe.visual(
        Box((L * 0.74, W * 1.04, H * 0.10)),
        origin=Origin(xyz=(-L * 0.03, 0, H * 1.04)),
        material="accent",
        name="cabin_roof_cap",
    )
    for side_name, y in (("port", W * 0.535), ("starboard", -W * 0.535)):
        airframe.visual(
            Box((L * 0.62, 0.040, H * 0.055)),
            origin=Origin(xyz=(-L * 0.08, y, H * 0.36)),
            material="dark",
            name=f"{side_name}_lower_longeron",
        )
        airframe.visual(
            Box((L * 0.58, 0.040, H * 0.055)),
            origin=Origin(xyz=(-L * 0.10, y, H * 0.98)),
            material="accent",
            name=f"{side_name}_roof_longeron",
        )
    airframe.visual(
        Box((L * 0.24, W * 0.68, H * 0.30)),
        origin=Origin(xyz=(L * 0.47, 0, H * 0.36)),
        material="accent",
        name="nose_chin_fairing",
    )
    airframe.visual(
        Box((L * 0.30, W * 0.76, H * 0.18)),
        origin=Origin(xyz=(L * 0.38, 0, H * 0.88)),
        material="dark",
        name="windshield_brow_frame",
    )
    airframe.visual(
        Box((L * 0.32, W * 0.72, H * 0.32)),
        origin=Origin(xyz=(-L * 0.10, 0, H * 1.12)),
        material="accent",
        name="engine_cowling",
    )
    for side_name, y in (("port", W * 0.43), ("starboard", -W * 0.43)):
        airframe.visual(
            Box((L * 0.12, 0.055, H * 0.15)),
            origin=Origin(xyz=(-L * 0.12, y, H * 1.16)),
            material="dark",
            name=f"{side_name}_engine_intake",
        )
        airframe.visual(
            Cylinder(radius=0.045, length=0.22),
            origin=Origin(xyz=(-L * 0.31, y, H * 1.11), rpy=_cyl_x()),
            material="dark",
            name=f"{side_name}_exhaust_stub",
        )
    airframe.visual(
        Cylinder(radius=0.13, length=0.46),
        origin=Origin(xyz=(r.mast_origin[0], 0, H * 1.35)),
        material="metal",
        name="mast_fairing",
    )
    airframe.visual(
        Cylinder(radius=0.20, length=0.09),
        origin=Origin(xyz=(r.mast_origin[0], 0, H * 1.23)),
        material="dark",
        name="transmission_bearing_ring",
    )
    tail_boom_len = T + 0.70
    airframe.visual(
        Cylinder(radius=0.17, length=tail_boom_len),
        origin=Origin(xyz=(-L * 0.42 - tail_boom_len * 0.5, 0, H * 0.90), rpy=_cyl_x()),
        material="accent",
        name="tail_boom",
    )
    airframe.visual(
        Box((0.46, W * 0.50, H * 0.42)),
        origin=Origin(xyz=(-L * 0.48, 0, H * 0.88)),
        material="accent",
        name="tail_root_fairing",
    )
    airframe.visual(
        Box((T * 0.28, W * 0.18, H * 0.13)),
        origin=Origin(xyz=(-L * 0.47 - T * 0.36, 0, H * 1.02)),
        material="accent",
        name="boom_spine",
    )
    tail_drive_cover_x = -L * 0.45 - T * 0.46
    tail_drive_cover_z = H * 1.055
    tail_drive_cover_len = T * 0.72
    _add_tube_mesh(
        airframe,
        model,
        [
            (tail_drive_cover_x - tail_drive_cover_len * 0.50, 0, tail_drive_cover_z),
            (tail_drive_cover_x - tail_drive_cover_len * 0.15, 0, tail_drive_cover_z + H * 0.018),
            (tail_drive_cover_x + tail_drive_cover_len * 0.50, 0, tail_drive_cover_z),
        ],
        radius=max(0.018, H * 0.025),
        material="dark",
        name="tail_drive_shaft_cover",
        samples_per_segment=12,
        radial_segments=14,
    )
    _add_tail_drive_supports(
        airframe,
        r,
        cover_x=tail_drive_cover_x,
        cover_z=tail_drive_cover_z,
        cover_length=tail_drive_cover_len,
    )
    airframe.visual(
        Box((0.20, W * 0.38, H * 0.92)),
        origin=Origin(xyz=(r.tail_rotor_origin[0] + 0.16, 0, H * 1.15)),
        material="accent",
        name="vertical_fin",
    )
    airframe.visual(
        Box((0.18, W * 0.30, H * 0.26)),
        origin=Origin(xyz=r.tail_rotor_origin),
        material="dark",
        name="tail_gearbox",
    )
    airframe.visual(
        Cylinder(radius=0.055, length=0.18),
        origin=Origin(xyz=(r.tail_rotor_origin[0] - 0.02, 0, r.tail_rotor_origin[2]), rpy=_cyl_x()),
        material="metal",
        name="tail_rotor_output_shaft",
    )
    airframe.visual(
        Box((W * 0.85, 0.14, 0.12)),
        origin=Origin(xyz=(r.tail_rotor_origin[0] + 0.35, 0, H * 0.80), rpy=(0, 0, math.pi / 2)),
        material="accent",
        name="horizontal_stabilizer",
    )
    for side_name, y in (("port", W * 0.27), ("starboard", -W * 0.27)):
        airframe.visual(
            Box((0.34, 0.055, H * 0.07)),
            origin=Origin(xyz=(r.tail_rotor_origin[0] + 0.28, y, H * 0.92)),
            material="metal",
            name=f"{side_name}_stabilizer_root_bracket",
        )
        _add_member(
            airframe,
            (r.tail_rotor_origin[0] + 0.18, y * 0.45, H * 1.06),
            (r.tail_rotor_origin[0] + 0.32, y, H * 0.82),
            0.025,
            "metal",
            name=f"{side_name}_tailplane_diagonal_strut",
        )
    for side, y in (("left", W * 0.68), ("right", -W * 0.68)):
        _add_tube_mesh(
            airframe,
            model,
            [
                (-L * 0.43, y, 0.10),
                (-L * 0.18, y, 0.075),
                (L * 0.18, y, 0.075),
                (L * 0.36, y, 0.10),
            ],
            radius=0.035,
            material="dark",
            name=f"{side}_rounded_skid_overlay",
            samples_per_segment=10,
            radial_segments=14,
        )
        airframe.visual(
            Box((L * 0.70, 0.08, 0.08)),
            origin=Origin(xyz=(-L * 0.05, y, 0.08)),
            material="dark",
            name=f"{side}_skid",
        )
        for cap_idx, x in enumerate((-L * 0.42, L * 0.32)):
            airframe.visual(
                Sphere(radius=0.060),
                origin=Origin(xyz=(x, y, 0.08)),
                material="dark",
                name=f"{side}_skid_end_cap_{cap_idx}",
            )
        airframe.visual(
            Box((L * 0.58, 0.030, 0.030)),
            origin=Origin(xyz=(-L * 0.06, y, 0.15)),
            material="metal",
            name=f"{side}_skid_wear_strip",
        )
    for idx, x in enumerate((-L * 0.30, L * 0.18)):
        airframe.visual(
            Box((0.10, W * 1.50, 0.08)),
            origin=Origin(xyz=(x, 0, 0.14)),
            material="metal",
            name=f"skid_cross_tube_{idx}",
        )
        for side_name, y in (("left", W * 0.68), ("right", -W * 0.68)):
            airframe.visual(
                Box((0.075, 0.060, H * 0.46)),
                origin=Origin(xyz=(x, y, H * 0.26)),
                material="metal",
                name=f"{side_name}_skid_vertical_strut_{idx}",
            )
            airframe.visual(
                Box((0.18, 0.052, H * 0.07)),
                origin=Origin(xyz=(x + (0.07 if idx == 0 else -0.07), y, H * 0.48)),
                material="metal",
                name=f"{side_name}_skid_upper_brace_{idx}",
            )
            _add_member(
                airframe,
                (x, y, H * 0.48),
                (x + (0.12 if idx == 0 else -0.12), y, 0.12),
                0.020,
                "metal",
                name=f"{side_name}_skid_diagonal_tube_{idx}",
            )
    if r.doors_service_module == "service_and_crew_hinges":
        for side_name, y in (("right", W * 0.60), ("left", -W * 0.60)):
            airframe.visual(
                Cylinder(radius=0.026, length=H * 0.42),
                origin=Origin(xyz=(-L * 0.16, y, H * 0.39)),
                material="metal",
                name=f"{side_name}_service_hinge_mount",
            )
        airframe.visual(
            Cylinder(radius=0.030, length=H * 0.62),
            origin=Origin(xyz=(L * 0.34, W * 0.60, H * 0.56)),
            material="metal",
            name="crew_hinge_mount",
        )
    elif r.doors_service_module == "cockpit_hinges_and_cabin_slide":
        airframe.visual(
            Box((L * 0.48, 0.08, 0.08)),
            origin=Origin(xyz=(-L * 0.06, -W * 0.64, H * 0.98)),
            material="metal",
            name="starboard_cabin_track",
        )
        airframe.visual(
            Box((L * 0.38, 0.035, 0.040)),
            origin=Origin(xyz=(-L * 0.08, -W * 0.68, H * 0.26)),
            material="metal",
            name="starboard_cabin_lower_guide",
        )
        airframe.visual(
            Box((0.04, 0.06, H * 0.90)),
            origin=Origin(xyz=(L * 0.38, W * 0.60, H * 0.58)),
            material="metal",
            name="port_hinge_post",
        )
        airframe.visual(
            Box((0.04, 0.06, H * 0.90)),
            origin=Origin(xyz=(L * 0.38, -W * 0.60, H * 0.58)),
            material="metal",
            name="starboard_hinge_post",
        )
    _add_windows(airframe, r)
    _add_surface_detailing(airframe, r)
    _add_mission_profile_details(airframe, r)
    airframe.inertial = Inertial.from_geometry(Box((L + T, W * 1.55, H * 1.8)), mass=1800)
    return airframe


def _build_transport_airframe(model: ArticulatedObject, r: ResolvedSingleRotorHelicopterConfig):
    airframe = model.part("airframe")
    L, W, H, T = r.fuselage_length, r.fuselage_width, r.fuselage_height, r.tail_length
    airframe.visual(
        _mesh(model, _fuselage_shell_geometry(r), "transport_fuselage_shell"),
        material="body",
        name="fuselage_shell",
    )
    airframe.visual(
        Box((L * 0.58, W * 0.68, H * 0.58)),
        origin=Origin(xyz=(-L * 0.08, 0, H * 0.58)),
        material="body",
        name="cabin_core",
    )
    for side_name, y in (("port", W * 0.56), ("starboard", -W * 0.56)):
        airframe.visual(
            Box((L * 0.74, 0.050, H * 0.070)),
            origin=Origin(xyz=(-L * 0.08, y, H * 0.32)),
            material="dark",
            name=f"{side_name}_lower_longeron",
        )
        airframe.visual(
            Box((L * 0.66, 0.045, H * 0.060)),
            origin=Origin(xyz=(-L * 0.10, y, H * 1.02)),
            material="accent",
            name=f"{side_name}_roof_longeron",
        )
    airframe.visual(
        Box((L * 0.90, W * 1.12, 0.20)),
        origin=Origin(xyz=(-L * 0.03, 0, H * 0.13)),
        material="accent",
        name="cabin_floor",
    )
    airframe.visual(
        Box((L * 0.28, W * 0.80, H * 0.28)),
        origin=Origin(xyz=(L * 0.43, 0, H * 0.35)),
        material="accent",
        name="nose_lower",
    )
    airframe.visual(
        Sphere(radius=W * 0.24),
        origin=Origin(xyz=(L * 0.66, 0, H * 0.60)),
        material="warning",
        name="nose_radome",
    )
    airframe.visual(
        Box((L * 0.34, W * 0.78, H * 0.18)),
        origin=Origin(xyz=(L * 0.34, 0, H * 0.98)),
        material="dark",
        name="nose_upper_windscreen_frame",
    )
    airframe.visual(
        Box((L * 0.36, W * 0.82, H * 0.30)),
        origin=Origin(xyz=(-L * 0.10, 0, H * 1.12)),
        material="accent",
        name="engine_cowling",
    )
    for side_name, y in (("port", W * 0.43), ("starboard", -W * 0.43)):
        airframe.visual(
            Box((L * 0.14, 0.060, H * 0.16)),
            origin=Origin(xyz=(-L * 0.16, y, H * 1.17)),
            material="dark",
            name=f"{side_name}_engine_intake",
        )
        airframe.visual(
            Cylinder(radius=0.050, length=0.24),
            origin=Origin(xyz=(-L * 0.34, y, H * 1.12), rpy=_cyl_x()),
            material="dark",
            name=f"{side_name}_exhaust_stub",
        )
    airframe.visual(
        Cylinder(radius=0.12, length=0.60),
        origin=Origin(xyz=(r.mast_origin[0], 0, H * 1.42)),
        material="metal",
        name="tall_mast_fairing",
    )
    airframe.visual(
        Cylinder(radius=0.22, length=0.10),
        origin=Origin(xyz=(r.mast_origin[0], 0, H * 1.30)),
        material="dark",
        name="transmission_bearing_ring",
    )
    tail_boom_len = T + 0.70
    airframe.visual(
        Cylinder(radius=0.20, length=tail_boom_len),
        origin=Origin(xyz=(-L * 0.43 - tail_boom_len * 0.5, 0, H * 0.88), rpy=_cyl_x()),
        material="accent",
        name="tail_boom",
    )
    airframe.visual(
        Box((0.56, W * 0.54, H * 0.42)),
        origin=Origin(xyz=(-L * 0.50, 0, H * 0.88)),
        material="accent",
        name="tail_root_fairing",
    )
    airframe.visual(
        Box((T * 0.34, W * 0.22, H * 0.16)),
        origin=Origin(xyz=(-L * 0.48 - T * 0.34, 0, H * 1.03)),
        material="accent",
        name="boom_spine",
    )
    tail_drive_cover_x = -L * 0.48 - T * 0.44
    tail_drive_cover_z = H * 1.045
    tail_drive_cover_len = T * 0.72
    _add_tube_mesh(
        airframe,
        model,
        [
            (tail_drive_cover_x - tail_drive_cover_len * 0.50, 0, tail_drive_cover_z),
            (tail_drive_cover_x - tail_drive_cover_len * 0.12, 0, tail_drive_cover_z + H * 0.020),
            (tail_drive_cover_x + tail_drive_cover_len * 0.50, 0, tail_drive_cover_z),
        ],
        radius=max(0.020, H * 0.026),
        material="dark",
        name="tail_drive_shaft_cover",
        samples_per_segment=12,
        radial_segments=14,
    )
    _add_tail_drive_supports(
        airframe,
        r,
        cover_x=tail_drive_cover_x,
        cover_z=tail_drive_cover_z,
        cover_length=tail_drive_cover_len,
    )
    airframe.visual(
        Box((0.24, W * 0.30, H * 1.15)),
        origin=Origin(xyz=(r.tail_rotor_origin[0] + 0.16, 0, H * 1.28)),
        material="accent",
        name="fin",
    )
    airframe.visual(
        Box((0.28, W * 0.28, H * 0.22)),
        origin=Origin(xyz=(r.tail_rotor_origin[0] + 0.16, 0, H * 1.92)),
        material="warning",
        name="fin_cap",
    )
    airframe.visual(
        Box((0.44, W * 0.38, H * 0.24)),
        origin=Origin(xyz=(r.tail_rotor_origin[0], 0, r.tail_rotor_origin[2])),
        material="dark",
        name="tail_gearbox",
    )
    airframe.visual(
        Cylinder(radius=0.060, length=0.20),
        origin=Origin(xyz=(r.tail_rotor_origin[0] - 0.04, 0, r.tail_rotor_origin[2]), rpy=_cyl_x()),
        material="metal",
        name="tail_rotor_output_shaft",
    )
    for side_name, y in (("port", W * 0.20), ("starboard", -W * 0.20)):
        _add_member(
            airframe,
            (r.tail_rotor_origin[0] + 0.11, y * 0.35, H * 1.12),
            (r.tail_rotor_origin[0] + 0.16, y, r.tail_rotor_origin[2] - H * 0.10),
            0.030,
            "metal",
            name=f"{side_name}_tail_gearbox_drag_strut",
        )
    for side_name, y in (("port", W * 0.77), ("starboard", -W * 0.77)):
        wheel_x = -L * 0.20
        wheel_z = -H * 0.34
        side_sign = 1.0 if y > 0 else -1.0
        main_fork_crown_z = wheel_z + 0.28
        main_fork_crown_h = H * 0.055
        main_strut_bottom_z = main_fork_crown_z + main_fork_crown_h * 0.5
        main_strut_top_z = H * 0.47
        main_strut_h = main_strut_top_z - main_strut_bottom_z
        airframe.visual(
            Box((L * 0.34, W * 0.42, H * 0.20)),
            origin=Origin(xyz=(wheel_x, side_sign * W * 0.66, H * 0.20)),
            material="dark",
            name=f"{side_name}_gear_root_fairing",
        )
        airframe.visual(
            Box((L * 0.24, W * 0.48, H * 0.075)),
            origin=Origin(xyz=(wheel_x, side_sign * W * 0.64, H * 0.36)),
            material="metal",
            name=f"{side_name}_gear_root_beam",
        )
        airframe.visual(
            Box((L * 0.30, 0.060, H * 0.38)),
            origin=Origin(xyz=(wheel_x, side_sign * W * 0.565, H * 0.28)),
            material="metal",
            name=f"{side_name}_gear_sidewall_web",
        )
        airframe.visual(
            Box((L * 0.30, 0.26, H * 0.28)),
            origin=Origin(xyz=(wheel_x, y, H * 0.18)),
            material="dark",
            name=f"{side_name}_sponson",
        )
        airframe.visual(
            Box((L * 0.20, 0.16, H * 0.16)),
            origin=Origin(xyz=(wheel_x, y, H * 0.42)),
            material="metal",
            name=f"{side_name}_gear_trunnion",
        )
        airframe.visual(
            Box((0.095, 0.095, main_strut_h)),
            origin=Origin(xyz=(wheel_x, y, (main_strut_top_z + main_strut_bottom_z) * 0.5)),
            material="metal",
            name=f"{side_name}_main_gear_strut",
        )
        airframe.visual(
            Box((0.08, 0.08, H * 0.34)),
            origin=Origin(xyz=(-L * 0.12, y, H * 0.03)),
            material="metal",
            name=f"{side_name}_drag_brace",
        )
        airframe.visual(
            Box((0.16, 0.20, H * 0.055)),
            origin=Origin(xyz=(wheel_x, y, main_fork_crown_z)),
            material="metal",
            name=f"{side_name}_main_fork_crown",
        )
        main_cap_outer_half_width = 0.12 * 0.54 + 0.018 * 0.5
        for fork_side, sign in (("outer", 1.0), ("inner", -1.0)):
            airframe.visual(
                Box((0.12, 0.045, H * 0.045)),
                origin=Origin(
                    xyz=(wheel_x, y + sign * (main_cap_outer_half_width + 0.045 * 0.5), wheel_z)
                ),
                material="metal",
                name=f"{side_name}_main_axle_pin_{fork_side}",
            )
            airframe.visual(
                Box((0.055, 0.030, H * 0.40)),
                origin=Origin(
                    xyz=(
                        wheel_x,
                        y + sign * (main_cap_outer_half_width + 0.030 * 0.5),
                        wheel_z + H * 0.075,
                    )
                ),
                material="dark",
                name=f"{side_name}_main_wheel_fork_{fork_side}",
            )
        torque_link_y = y + (0.120 if y > 0 else -0.120)
        _add_member(
            airframe,
            (wheel_x, torque_link_y, main_strut_bottom_z + H * 0.05),
            (wheel_x, torque_link_y, wheel_z + H * 0.10),
            0.024,
            "metal",
            name=f"{side_name}_main_lower_torque_link",
        )
        _add_member(
            airframe,
            (-L * 0.30, y, H * 0.35),
            (wheel_x, y, -H * 0.13),
            0.026,
            "metal",
            name=f"{side_name}_forward_oleo_link",
        )
        _add_member(
            airframe,
            (-L * 0.10, y, H * 0.32),
            (wheel_x, y, -H * 0.13),
            0.026,
            "metal",
            name=f"{side_name}_aft_oleo_link",
        )
    airframe.visual(
        Box((0.09, 0.09, H * 0.36)),
        origin=Origin(xyz=(L * 0.48, 0, 0.0)),
        material="metal",
        name="nose_gear_strut",
    )
    airframe.visual(
        Box((0.34, W * 0.34, H * 0.13)),
        origin=Origin(xyz=(L * 0.48, 0, H * 0.12)),
        material="dark",
        name="nose_gear_bay",
    )
    airframe.visual(
        Box((0.24, W * 0.24, H * 0.16)),
        origin=Origin(xyz=(L * 0.48, 0, H * 0.02)),
        material="metal",
        name="nose_gear_upper_socket",
    )
    airframe.visual(
        Box((0.20, 0.44, H * 0.055)),
        origin=Origin(xyz=(L * 0.48, 0, -H * 0.14)),
        material="metal",
        name="nose_fork_crown",
    )
    airframe.visual(
        Cylinder(radius=0.035, length=0.24),
        origin=Origin(xyz=(L * 0.48, 0, -H * 0.34), rpy=_cyl_y()),
        material="metal",
        name="nose_axle_sleeve",
    )
    nose_cap_outer_half_width = 0.34 * 0.54 + 0.018 * 0.5
    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        airframe.visual(
            Box((0.14, 0.050, H * 0.045)),
            origin=Origin(
                xyz=(L * 0.48, sign * (nose_cap_outer_half_width + 0.050 * 0.5), -H * 0.34)
            ),
            material="metal",
            name=f"nose_axle_pin_{side_name}",
        )
        airframe.visual(
            Box((0.060, 0.040, H * 0.35)),
            origin=Origin(
                xyz=(L * 0.48, sign * (nose_cap_outer_half_width + 0.040 * 0.5), -H * 0.28)
            ),
            material="dark",
            name=f"nose_wheel_fork_{side_name}",
        )
    _add_member(
        airframe,
        (L * 0.39, 0, H * 0.11),
        (L * 0.48, 0, -H * 0.14),
        0.028,
        "metal",
        name="nose_gear_forward_brace",
    )
    _add_member(
        airframe,
        (L * 0.57, 0, H * 0.09),
        (L * 0.48, 0, -H * 0.14),
        0.028,
        "metal",
        name="nose_gear_aft_brace",
    )
    if r.doors_service_module == "service_and_crew_hinges":
        for side_name, y in (("right", W * 0.60), ("left", -W * 0.60)):
            airframe.visual(
                Cylinder(radius=0.026, length=H * 0.42),
                origin=Origin(xyz=(-L * 0.16, y, H * 0.39)),
                material="metal",
                name=f"{side_name}_service_hinge_mount",
            )
        airframe.visual(
            Cylinder(radius=0.030, length=H * 0.62),
            origin=Origin(xyz=(L * 0.34, W * 0.60, H * 0.56)),
            material="metal",
            name="crew_hinge_mount",
        )
    elif r.doors_service_module == "cockpit_hinges_and_cabin_slide":
        airframe.visual(
            Box((L * 0.48, 0.08, 0.08)),
            origin=Origin(xyz=(-L * 0.06, -W * 0.64, H * 0.98)),
            material="metal",
            name="starboard_cabin_track",
        )
        airframe.visual(
            Box((0.04, 0.06, H * 0.90)),
            origin=Origin(xyz=(L * 0.38, W * 0.60, H * 0.58)),
            material="metal",
            name="port_hinge_post",
        )
        airframe.visual(
            Box((0.04, 0.06, H * 0.90)),
            origin=Origin(xyz=(L * 0.38, -W * 0.60, H * 0.58)),
            material="metal",
            name="starboard_hinge_post",
        )
    _add_windows(airframe, r)
    _add_surface_detailing(airframe, r)
    _add_mission_profile_details(airframe, r)
    airframe.inertial = Inertial.from_geometry(Box((L + T, W * 1.7, H * 1.9)), mass=2600)
    return airframe


def _build_airframe(model: ArticulatedObject, r: ResolvedSingleRotorHelicopterConfig):
    if r.airframe_landing_module == "offshore_wheeled_airframe":
        return _build_transport_airframe(model, r)
    return _build_fire_airframe(model, r)


def _build_transport_wheel_articulations(
    model: ArticulatedObject, r: ResolvedSingleRotorHelicopterConfig
) -> None:
    if r.airframe_landing_module != "offshore_wheeled_airframe":
        return
    L, W, H = r.fuselage_length, r.fuselage_width, r.fuselage_height
    wheel_specs = (
        ("port_main_wheel", -L * 0.20, W * 0.77, -H * 0.34, 0.22, 0.12, 65.0),
        ("starboard_main_wheel", -L * 0.20, -W * 0.77, -H * 0.34, 0.22, 0.12, 65.0),
        ("nose_wheel", L * 0.48, 0.0, -H * 0.34, 0.18, 0.34, 45.0),
    )
    for name, x, y, z, radius, width, effort in wheel_specs:
        wheel = model.part(name)
        wheel.visual(
            Cylinder(radius=radius, length=width),
            origin=Origin(rpy=_cyl_y()),
            material="tire",
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=radius * 0.44, length=width * 1.06),
            origin=Origin(rpy=_cyl_y()),
            material="metal",
            name="hub",
        )
        for side_name, y_offset in (("outer", width * 0.54), ("inner", -width * 0.54)):
            wheel.visual(
                Cylinder(radius=radius * 0.30, length=0.018),
                origin=Origin(xyz=(0, y_offset, 0), rpy=_cyl_y()),
                material="metal",
                name=f"{side_name}_hub_cap",
            )
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent="airframe",
            child=wheel,
            origin=Origin(xyz=(x, y, z)),
            axis=(0, 1, 0),
            motion_limits=MotionLimits(effort=effort, velocity=18),
        )


def _build_main_rotor(model: ArticulatedObject, r: ResolvedSingleRotorHelicopterConfig) -> None:
    rotor = model.part("main_rotor")
    hub_radius = 0.20 if r.main_rotor_module == "tall_mast_transport_rotor" else 0.18
    mast_height = 0.28 if r.main_rotor_module == "tall_mast_transport_rotor" else 0.20
    rotor.visual(
        Cylinder(radius=0.10, length=mast_height),
        origin=Origin(xyz=(0, 0, mast_height * 0.5)),
        material="metal",
        name="mast",
    )
    rotor.visual(
        Cylinder(radius=hub_radius * 0.86, length=0.035),
        origin=Origin(xyz=(0, 0, mast_height - 0.025)),
        material="metal",
        name="swashplate",
    )
    rotor.visual(
        Cylinder(radius=hub_radius, length=0.14),
        origin=Origin(xyz=(0, 0, mast_height + 0.02)),
        material="rotor",
        name="hub",
    )
    rotor.visual(
        Cylinder(radius=hub_radius * 0.72, length=0.10),
        origin=Origin(xyz=(0, 0, mast_height + 0.10)),
        material="metal",
        name="hub_cap",
    )
    blade_len = r.main_rotor_radius - hub_radius * 1.2
    for index in range(r.main_blade_count):
        angle = index * math.tau / r.main_blade_count
        cuff_offset = hub_radius + 0.22
        blade_offset = hub_radius + blade_len * 0.50
        rotor.visual(
            Box((0.52, 0.22, 0.055)),
            origin=Origin(
                xyz=(
                    math.cos(angle) * cuff_offset,
                    math.sin(angle) * cuff_offset,
                    mast_height + 0.02,
                ),
                rpy=(0, 0, angle),
            ),
            material="rotor",
            name=f"cuff_{index}",
        )
        control_radius = hub_radius * 0.76
        rotor.visual(
            Box((0.18, 0.035, 0.018)),
            origin=Origin(
                xyz=(
                    math.cos(angle) * (hub_radius * 0.76),
                    math.sin(angle) * (hub_radius * 0.76),
                    mast_height - 0.020,
                ),
                rpy=(0, 0, angle),
            ),
            material="metal",
            name=f"swashplate_pitch_arm_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.014, length=0.205),
            origin=Origin(
                xyz=(
                    math.cos(angle) * control_radius,
                    math.sin(angle) * control_radius,
                    mast_height + 0.040,
                ),
            ),
            material="metal",
            name=f"pitch_link_{index}",
        )
        rotor.visual(
            Box((0.11, 0.030, 0.045)),
            origin=Origin(
                xyz=(
                    math.cos(angle) * (hub_radius + 0.24),
                    math.sin(angle) * (hub_radius + 0.24),
                    mast_height + 0.040,
                ),
                rpy=(0, 0, angle),
            ),
            material="metal",
            name=f"blade_pitch_horn_{index}",
        )
        rotor.visual(
            Box((0.28, 0.045, 0.035)),
            origin=Origin(
                xyz=(
                    math.cos(angle + 0.16) * (hub_radius + 0.32),
                    math.sin(angle + 0.16) * (hub_radius + 0.32),
                    mast_height - 0.015,
                ),
                rpy=(0, 0, angle),
            ),
            material="metal",
            name=f"lead_lag_damper_{index}",
        )
        rotor.visual(
            _mesh(
                model,
                _tapered_blade_geometry(
                    blade_len,
                    root_chord=0.20,
                    tip_chord=0.13,
                    thickness=0.040,
                    sweep=0.10,
                ),
                f"main_blade_{index}_mesh",
            ),
            origin=Origin(
                xyz=(
                    math.cos(angle) * blade_offset,
                    math.sin(angle) * blade_offset,
                    mast_height + 0.02,
                ),
                rpy=(0, 0, angle),
            ),
            material="dark",
            name=f"main_blade_{index}",
        )
        rotor.visual(
            Box((0.18, 0.20, 0.040)),
            origin=Origin(
                xyz=(
                    math.cos(angle) * (hub_radius + blade_len - 0.10),
                    math.sin(angle) * (hub_radius + blade_len - 0.10),
                    mast_height + 0.025,
                ),
                rpy=(0, 0, angle),
            ),
            material="warning",
            name=f"blade_tip_{index}",
        )
        for bolt_index, lateral in enumerate((-0.075, 0.075)):
            rotor.visual(
                Sphere(radius=0.024),
                origin=Origin(
                    xyz=(
                        math.cos(angle) * (hub_radius + 0.22) - math.sin(angle) * lateral,
                        math.sin(angle) * (hub_radius + 0.22) + math.cos(angle) * lateral,
                        mast_height + 0.075,
                    )
                ),
                material="metal",
                name=f"blade_root_bolt_{index}_{bolt_index}",
            )
    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent="airframe",
        child=rotor,
        origin=Origin(xyz=r.mast_origin),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=300, velocity=35),
    )


def _build_tail_rotor(model: ArticulatedObject, r: ResolvedSingleRotorHelicopterConfig) -> None:
    rotor = model.part("tail_rotor")
    rotor.visual(
        Cylinder(radius=0.09, length=0.16),
        origin=Origin(xyz=(-0.08, 0, 0), rpy=_cyl_x()),
        material="metal",
        name="hub",
    )
    rotor.visual(
        Cylinder(radius=0.035, length=0.21),
        origin=Origin(xyz=(-0.085, 0, 0), rpy=_cyl_x()),
        material="metal",
        name="input_shaft",
    )
    rotor.visual(
        Box((0.22, 0.14, 0.14)),
        origin=Origin(xyz=(-0.17, 0, 0)),
        material="rotor",
        name="hub_block",
    )
    blade_len = r.tail_rotor_radius
    for index in range(r.tail_blade_count):
        angle = index * math.tau / r.tail_blade_count
        rotor.visual(
            Box((0.045, 0.16, 0.070)),
            origin=Origin(
                xyz=(
                    -0.30,
                    math.cos(angle) * 0.15,
                    math.sin(angle) * 0.15,
                ),
                rpy=(angle, 0, 0),
            ),
            material="rotor",
            name=f"tail_cuff_{index}",
        )
        rotor.visual(
            _mesh(
                model,
                _tapered_blade_geometry(
                    blade_len,
                    root_chord=0.10,
                    tip_chord=0.055,
                    thickness=0.045,
                    sweep=0.025,
                ).rotate_z(math.pi / 2.0),
                f"tail_blade_{index}_mesh",
            ),
            origin=Origin(
                xyz=(
                    -0.30,
                    math.cos(angle) * (blade_len * 0.5 + 0.08),
                    math.sin(angle) * (blade_len * 0.5 + 0.08),
                ),
                rpy=(angle, 0, 0),
            ),
            material="dark",
            name=f"tail_blade_{index}",
        )
        rotor.visual(
            Box((0.018, blade_len * 0.48, 0.024)),
            origin=Origin(
                xyz=(
                    -0.33,
                    math.cos(angle) * (blade_len * 0.28 + 0.07),
                    math.sin(angle) * (blade_len * 0.28 + 0.07),
                ),
                rpy=(angle, 0, 0),
            ),
            material="metal",
            name=f"tail_pitch_link_{index}",
        )
    if r.tail_rotor_module == "fin_mounted_tail_rotor":
        rotor.visual(
            Cylinder(radius=0.11, length=0.05),
            origin=Origin(xyz=(-0.13, 0, 0), rpy=_cyl_x()),
            material="warning",
            name="fin_side_plate",
        )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent="airframe",
        child=rotor,
        origin=Origin(xyz=r.tail_rotor_origin),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=90, velocity=55),
    )


def _door_panel(part, *, side_sign: float, width: float, height: float, glass: bool) -> None:
    part.visual(
        Cylinder(radius=0.024, length=0.090),
        origin=Origin(rpy=_cyl_y()),
        material="metal",
        name="hinge_root_knuckle",
    )
    part.visual(
        Box((width, 0.055, height)),
        origin=Origin(xyz=(-width * 0.56, side_sign * 0.010, height * 0.50)),
        material="accent",
        name="panel",
    )
    part.visual(
        Box((width * 0.96, 0.020, height * 0.050)),
        origin=Origin(xyz=(-width * 0.45, side_sign * 0.040, height * 0.94)),
        material="metal",
        name="top_frame",
    )
    part.visual(
        Box((width * 0.96, 0.020, height * 0.050)),
        origin=Origin(xyz=(-width * 0.45, side_sign * 0.040, height * 0.06)),
        material="metal",
        name="bottom_frame",
    )
    part.visual(
        Box((width * 0.050, 0.020, height * 0.86)),
        origin=Origin(xyz=(-width * 0.90, side_sign * 0.040, height * 0.50)),
        material="metal",
        name="aft_frame",
    )
    for hinge_idx, z in enumerate((height * 0.24, height * 0.52, height * 0.78)):
        part.visual(
            Cylinder(radius=0.022, length=0.070),
            origin=Origin(xyz=(0.0, side_sign * 0.030, z), rpy=_cyl_y()),
            material="metal",
            name=f"hinge_knuckle_{hinge_idx}",
        )
    if glass:
        part.visual(
            Box((width * 0.48, 0.030, height * 0.34)),
            origin=Origin(xyz=(-width * 0.42, side_sign * 0.025, height * 0.70)),
            material="glass",
            name="window",
        )
    part.visual(
        Cylinder(radius=0.015, length=0.055),
        origin=Origin(xyz=(-width * 0.70, side_sign * 0.035, height * 0.44), rpy=_cyl_y()),
        material="dark",
        name="handle",
    )


def _build_service_doors(model: ArticulatedObject, r: ResolvedSingleRotorHelicopterConfig) -> None:
    L, W, H = r.fuselage_length, r.fuselage_width, r.fuselage_height
    if r.doors_service_module == "none":
        return
    if r.doors_service_module == "service_and_crew_hinges":
        specs = (
            (
                "right_service_door",
                1.0,
                -L * 0.16,
                W * 0.60,
                H * 0.18,
                H * 0.44,
                (0, 0, 1),
                1.65,
            ),
            (
                "left_service_door",
                -1.0,
                -L * 0.16,
                -W * 0.60,
                H * 0.18,
                H * 0.44,
                (0, 0, -1),
                1.65,
            ),
            ("crew_door", 1.0, L * 0.34, W * 0.60, H * 0.22, H * 0.82, (0, 0, 1), 1.25),
        )
        for name, sign, x, y, z, door_h, axis, upper in specs:
            door = model.part(name)
            _door_panel(
                door,
                side_sign=sign,
                width=0.72 if name != "crew_door" else 0.86,
                height=door_h,
                glass=name == "crew_door",
            )
            model.articulation(
                f"{name}_hinge",
                ArticulationType.REVOLUTE,
                parent="airframe",
                child=door,
                origin=Origin(xyz=(x, y, z)),
                axis=axis,
                motion_limits=MotionLimits(effort=10, velocity=1.2, lower=0, upper=upper),
            )
        return
    for name, sign, y, axis in (
        ("port_cockpit_door", 1.0, W * 0.60, (0, 0, -1)),
        ("starboard_cockpit_door", -1.0, -W * 0.60, (0, 0, 1)),
    ):
        door = model.part(name)
        _door_panel(door, side_sign=sign, width=0.62, height=H * 0.78, glass=True)
        model.articulation(
            f"{name}_hinge",
            ArticulationType.REVOLUTE,
            parent="airframe",
            child=door,
            origin=Origin(xyz=(L * 0.38, y, H * 0.24)),
            axis=axis,
            motion_limits=MotionLimits(effort=24, velocity=1.1, lower=0, upper=1.20),
        )
    cabin = model.part("starboard_cabin_door")
    cabin.visual(
        Box((L * 0.34, 0.055, H * 0.66)),
        origin=Origin(xyz=(-L * 0.17, 0.030, -H * 0.38)),
        material="accent",
        name="sliding_panel",
    )
    cabin.visual(
        Box((L * 0.20, 0.030, H * 0.28)),
        origin=Origin(xyz=(-L * 0.17, 0.015, -H * 0.24)),
        material="glass",
        name="sliding_window",
    )
    cabin.visual(
        Box((L * 0.34, 0.035, 0.045)),
        origin=Origin(xyz=(-L * 0.17, 0.0, 0.0)),
        material="metal",
        name="door_track_runner",
    )
    model.articulation(
        "starboard_cabin_door_slide",
        ArticulationType.PRISMATIC,
        parent="airframe",
        child=cabin,
        origin=Origin(xyz=(L * 0.12, -W * 0.64, H * 0.98)),
        axis=(-1, 0, 0),
        motion_limits=MotionLimits(effort=90, velocity=0.7, lower=0, upper=L * 0.28),
    )


def build_single_rotor_helicopter(
    config: SingleRotorHelicopterConfig,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name="single_rotor_helicopter", assets=assets)
    for name, rgba in r.palette.items():
        model.material(name, rgba=rgba)
    _build_airframe(model, r)
    _build_transport_wheel_articulations(model, r)
    _build_main_rotor(model, r)
    _build_tail_rotor(model, r)
    _build_service_doors(model, r)
    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_single_rotor_helicopter(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_single_rotor_helicopter(config_from_seed(seed), assets=assets)


def slot_choices_for_config(r: ResolvedSingleRotorHelicopterConfig) -> list[tuple[str, str]]:
    return [
        ("airframe_landing", r.airframe_landing_module),
        ("main_rotor", r.main_rotor_module),
        ("tail_rotor", r.tail_rotor_module),
        ("doors_service", r.doors_service_module),
        ("mission_profile", r.mission_profile),
    ]


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


def run_single_rotor_helicopter_tests(
    object_model: ArticulatedObject,
    config: SingleRotorHelicopterConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    names = {p.name for p in object_model.parts}
    joints = {j.name: j for j in object_model.articulations}
    ctx.check("airframe_present", "airframe" in names)
    ctx.check("main_rotor_present", "main_rotor" in names)
    ctx.check("tail_rotor_present", "tail_rotor" in names)
    ctx.check("one_main_rotor_spin", "main_rotor_spin" in joints)
    ctx.check("one_tail_rotor_spin", "tail_rotor_spin" in joints)
    if "main_rotor_spin" in joints:
        ctx.check(
            "main_rotor_axis_vertical", tuple(joints["main_rotor_spin"].axis) == (0.0, 0.0, 1.0)
        )
        ctx.check(
            "main_rotor_continuous",
            joints["main_rotor_spin"].articulation_type == ArticulationType.CONTINUOUS,
        )
    if "tail_rotor_spin" in joints:
        ctx.check(
            "tail_rotor_axis_horizontal_x", tuple(joints["tail_rotor_spin"].axis) == (1.0, 0.0, 0.0)
        )
        ctx.check(
            "tail_rotor_continuous",
            joints["tail_rotor_spin"].articulation_type == ArticulationType.CONTINUOUS,
        )
    if "airframe" in names:
        airframe = object_model.get_part("airframe")
        airframe_visuals = {v.name for v in airframe.visuals}
        expected_profile_visuals = {
            "fire_utility": {"belly_water_tank", "tank_pickup_manifold"},
            "rescue_hoist": {
                "rescue_hoist_pylon",
                "rescue_hoist_boom",
                "rescue_hoist_roof_clamp",
            },
            "agricultural_sprayer": {"chemical_tank", "spray_boom_crossbar"},
            "tour_bubble": {"panoramic_nose_bubble", "aft_luggage_pod"},
            "police_observer": {"nose_sensor_turret", "side_searchlight_barrel"},
            "offshore_float_pack": {
                "port_emergency_float_bag",
                "starboard_emergency_float_bag",
            },
        }
        ctx.check(
            "mission_profile_has_distinctive_visuals",
            expected_profile_visuals[r.mission_profile].issubset(airframe_visuals),
        )
        ctx.check(
            "tail_drive_cover_has_visible_supports",
            {
                "tail_drive_shaft_cover",
                "tail_drive_cover_stanchion_0",
                "tail_drive_cover_saddle_0",
            }.issubset(airframe_visuals),
        )
        for door_name in (
            "left_service_door",
            "right_service_door",
            "crew_door",
            "port_cockpit_door",
            "starboard_cockpit_door",
            "starboard_cabin_door",
        ):
            if door_name in names:
                ctx.allow_overlap(
                    airframe,
                    object_model.get_part(door_name),
                    reason=f"{door_name} hinge knuckles and closed panel sit flush in the airframe skin and sponson fairing.",
                )

        def _expect_airframe_child_mount(
            child_name: str,
            airframe_elem: str,
            child_elem: str,
            *,
            reason: str,
            tol: float = 0.035,
        ) -> None:
            if child_name not in names:
                return
            child = object_model.get_part(child_name)
            ctx.expect_contact(
                airframe,
                child,
                elem_a=airframe_elem,
                elem_b=child_elem,
                contact_tol=tol,
                name=f"{child_name}_{child_elem}_contacts_{airframe_elem}",
            )
            ctx.allow_overlap(
                airframe,
                child,
                elem_a=airframe_elem,
                elem_b=child_elem,
                reason=reason,
            )

        mast_mount = (
            "tall_mast_fairing"
            if r.airframe_landing_module == "offshore_wheeled_airframe"
            else "mast_fairing"
        )
        _expect_airframe_child_mount(
            "main_rotor",
            mast_mount,
            "mast",
            reason="The main rotor mast is seated inside the visible transmission fairing.",
            tol=0.03,
        )
        _expect_airframe_child_mount(
            "tail_rotor",
            "tail_gearbox",
            "hub",
            reason="The tail rotor hub is seated in the tail gearbox housing.",
            tol=0.04,
        )
        _expect_airframe_child_mount(
            "tail_rotor",
            "tail_gearbox",
            "hub_block",
            reason="The tail rotor hub block is visibly captured by the tail gearbox housing.",
            tol=0.04,
        )
        _expect_airframe_child_mount(
            "tail_rotor",
            "tail_gearbox",
            "input_shaft",
            reason="The tail rotor input shaft enters the gearbox bearing.",
            tol=0.035,
        )
        _expect_airframe_child_mount(
            "tail_rotor",
            "tail_rotor_output_shaft",
            "input_shaft",
            reason="The external tail output shaft enters the tail rotor input shaft.",
            tol=0.025,
        )
        _expect_airframe_child_mount(
            "tail_rotor",
            "tail_rotor_output_shaft",
            "hub",
            reason="The visible tail output shaft passes into the rotor hub bearing.",
            tol=0.025,
        )
        _expect_airframe_child_mount(
            "tail_rotor",
            "tail_rotor_output_shaft",
            "hub_block",
            reason="The tail output shaft passes through the square hub block at the gearbox face.",
            tol=0.025,
        )

        if r.doors_service_module == "service_and_crew_hinges":
            for child_name, hinge_elem in (
                ("right_service_door", "right_service_hinge_mount"),
                ("left_service_door", "left_service_hinge_mount"),
                ("crew_door", "crew_hinge_mount"),
            ):
                _expect_airframe_child_mount(
                    child_name,
                    hinge_elem,
                    "hinge_knuckle_1",
                    reason="Door hinge knuckles are captured on visible hinge posts.",
                    tol=0.065,
                )
                if child_name in names:
                    door = object_model.get_part(child_name)
                    for knuckle_idx in (0, 2):
                        ctx.allow_overlap(
                            airframe,
                            door,
                            elem_a=hinge_elem,
                            elem_b=f"hinge_knuckle_{knuckle_idx}",
                            reason="Door hinge knuckles are captured on visible hinge posts.",
                        )
                    ctx.allow_overlap(
                        airframe,
                        door,
                        elem_a=hinge_elem,
                        elem_b="hinge_root_knuckle",
                        reason="The door hinge root is sleeved around the fixed hinge post.",
                    )
                    ctx.allow_overlap(
                        airframe,
                        door,
                        elem_a="cabin_core",
                        elem_b="panel",
                        reason="Closed door panel sits flush in the fuselage skin opening.",
                    )
        elif r.doors_service_module == "cockpit_hinges_and_cabin_slide":
            for child_name, hinge_elem in (
                ("port_cockpit_door", "port_hinge_post"),
                ("starboard_cockpit_door", "starboard_hinge_post"),
            ):
                _expect_airframe_child_mount(
                    child_name,
                    hinge_elem,
                    "hinge_knuckle_1",
                    reason="Cockpit door hinge knuckles are captured on visible hinge posts.",
                    tol=0.065,
                )
                if child_name in names:
                    door = object_model.get_part(child_name)
                    for knuckle_idx in (0, 2):
                        ctx.allow_overlap(
                            airframe,
                            door,
                            elem_a=hinge_elem,
                            elem_b=f"hinge_knuckle_{knuckle_idx}",
                            reason="Cockpit door hinge knuckles are captured on visible hinge posts.",
                        )
                    ctx.allow_overlap(
                        airframe,
                        door,
                        elem_a=hinge_elem,
                        elem_b="hinge_root_knuckle",
                        reason="The cockpit door hinge root is sleeved around the fixed hinge post.",
                    )
                    ctx.allow_overlap(
                        airframe,
                        door,
                        elem_a="cabin_core",
                        elem_b="panel",
                        reason="Closed cockpit door panel sits flush in the fuselage skin opening.",
                    )
            _expect_airframe_child_mount(
                "starboard_cabin_door",
                "starboard_cabin_track",
                "door_track_runner",
                reason="The sliding cabin door runner is visibly retained by the upper rail.",
                tol=0.04,
            )
            if "starboard_cabin_door" in names:
                ctx.allow_overlap(
                    airframe,
                    object_model.get_part("starboard_cabin_door"),
                    elem_a="cabin_core",
                    elem_b="sliding_panel",
                    reason="Closed sliding door panel sits flush in the fuselage skin opening.",
                )
        if r.airframe_landing_module == "fire_utility_skid_airframe":
            for side in ("left", "right"):
                for idx in range(2):
                    strut_name = f"{side}_skid_vertical_strut_{idx}"
                    ctx.expect_contact(
                        airframe,
                        airframe,
                        elem_a=f"{side}_skid",
                        elem_b=strut_name,
                        contact_tol=0.015,
                        name=f"{strut_name}_lands_on_skid",
                    )
                    ctx.expect_contact(
                        airframe,
                        airframe,
                        elem_a=f"skid_cross_tube_{idx}",
                        elem_b=strut_name,
                        contact_tol=0.015,
                        name=f"{strut_name}_ties_into_cross_tube",
                    )
            ctx.check(
                "fire_airframe_has_skid_support_visuals",
                {
                    "left_skid",
                    "right_skid",
                    "skid_cross_tube_0",
                    "skid_cross_tube_1",
                    "left_skid_vertical_strut_0",
                    "right_skid_vertical_strut_0",
                }.issubset(airframe_visuals),
            )
        else:
            for side, wheel_name in (
                ("port", "port_main_wheel"),
                ("starboard", "starboard_main_wheel"),
            ):
                ctx.expect_contact(
                    airframe,
                    airframe,
                    elem_a="cabin_floor",
                    elem_b=f"{side}_gear_root_fairing",
                    contact_tol=0.020,
                    name=f"{side}_gear_root_fairing_attached_to_floor",
                )
                ctx.expect_contact(
                    airframe,
                    airframe,
                    elem_a=f"{side}_gear_root_fairing",
                    elem_b=f"{side}_sponson",
                    contact_tol=0.020,
                    name=f"{side}_gear_root_fairing_attached_to_sponson",
                )
                ctx.expect_contact(
                    airframe,
                    airframe,
                    elem_a=f"{side}_sponson",
                    elem_b=f"{side}_main_gear_strut",
                    contact_tol=0.015,
                    name=f"{side}_gear_strut_seated_in_sponson",
                )
                ctx.expect_contact(
                    airframe,
                    airframe,
                    elem_a=f"{side}_drag_brace",
                    elem_b=f"{side}_sponson",
                    contact_tol=0.025,
                    name=f"{side}_drag_brace_reaches_sponson",
                )
                ctx.check(f"{wheel_name}_part_present", wheel_name in names)
                ctx.check(f"{wheel_name}_spin_joint_present", f"{wheel_name}_spin" in joints)
                if f"{wheel_name}_spin" in joints:
                    ctx.check(
                        f"{wheel_name}_spin_axis_is_wheel_axle",
                        tuple(joints[f"{wheel_name}_spin"].axis) == (0.0, 1.0, 0.0),
                    )
                    ctx.check(
                        f"{wheel_name}_spin_continuous",
                        joints[f"{wheel_name}_spin"].articulation_type
                        == ArticulationType.CONTINUOUS,
                    )
                _expect_airframe_child_mount(
                    wheel_name,
                    f"{side}_main_wheel_fork_outer",
                    "tire",
                    reason="The main wheel tire is captured between the fixed landing-gear fork plates.",
                    tol=0.040,
                )
                if wheel_name in names:
                    ctx.allow_overlap(
                        airframe,
                        object_model.get_part(wheel_name),
                        elem_a=f"{side}_main_gear_strut",
                        elem_b="hub",
                        reason="The wheel hub is captured around the gear axle at the strut end.",
                    )
            ctx.expect_contact(
                airframe,
                airframe,
                elem_a="nose_gear_bay",
                elem_b="nose_gear_strut",
                contact_tol=0.020,
                name="nose_gear_strut_seated_in_bay",
            )
            ctx.check("nose_wheel_part_present", "nose_wheel" in names)
            ctx.check("nose_wheel_spin_joint_present", "nose_wheel_spin" in joints)
            if "nose_wheel_spin" in joints:
                ctx.check(
                    "nose_wheel_spin_axis_is_wheel_axle",
                    tuple(joints["nose_wheel_spin"].axis) == (0.0, 1.0, 0.0),
                )
                ctx.check(
                    "nose_wheel_spin_continuous",
                    joints["nose_wheel_spin"].articulation_type == ArticulationType.CONTINUOUS,
                )
            _expect_airframe_child_mount(
                "nose_wheel",
                "nose_wheel_fork_left",
                "tire",
                reason="The nose wheel tire is captured between the fixed nose fork plates.",
                tol=0.050,
            )
            if "nose_wheel" in names:
                ctx.allow_overlap(
                    airframe,
                    object_model.get_part("nose_wheel"),
                    elem_a="nose_gear_strut",
                    elem_b="hub",
                    reason="The nose wheel hub is captured around the nose axle at the strut end.",
                )
                ctx.allow_overlap(
                    airframe,
                    object_model.get_part("nose_wheel"),
                    elem_a="nose_axle_sleeve",
                    elem_b="tire",
                    reason="The fixed axle sleeve passes through the modeled wheel center.",
                )
                ctx.allow_overlap(
                    airframe,
                    object_model.get_part("nose_wheel"),
                    elem_a="nose_axle_sleeve",
                    elem_b="hub",
                    reason="The fixed axle sleeve is concentric with the rotating wheel hub.",
                )
        ctx.expect_contact(
            airframe,
            airframe,
            elem_a="tail_root_fairing",
            elem_b="tail_boom",
            contact_tol=0.015,
            name="tail_boom_seated_in_root_fairing",
        )
        if r.tail_rotor_module == "fin_mounted_tail_rotor":
            fin_elem = (
                "fin"
                if r.airframe_landing_module == "offshore_wheeled_airframe"
                else "vertical_fin"
            )
            ctx.expect_contact(
                airframe,
                airframe,
                elem_a="tail_boom",
                elem_b=fin_elem,
                contact_tol=0.040,
                name="tail_boom_reaches_tail_fin_support",
            )
            ctx.expect_contact(
                airframe,
                airframe,
                elem_a=fin_elem,
                elem_b="tail_gearbox",
                contact_tol=0.040,
                name="tail_fin_support_carries_tail_gearbox",
            )
            if "tail_rotor" in names:
                ctx.allow_overlap(
                    airframe,
                    object_model.get_part("tail_rotor"),
                    elem_a="tail_gearbox",
                    elem_b="fin_side_plate",
                    reason="The fin-side tail rotor plate is seated into the gearbox face.",
                )
                ctx.allow_overlap(
                    airframe,
                    object_model.get_part("tail_rotor"),
                    reason="Tail rotor input shaft and fin-side plate are intentionally captured by the tail gearbox.",
                )
                ctx.allow_overlap(
                    airframe,
                    object_model.get_part("tail_rotor"),
                    elem_a="tail_rotor_output_shaft",
                    elem_b="fin_side_plate",
                    reason="The tail rotor output shaft passes through the fin-side bearing plate.",
                )
        else:
            fin_elem = (
                "fin"
                if r.airframe_landing_module == "offshore_wheeled_airframe"
                else "vertical_fin"
            )
            ctx.expect_contact(
                airframe,
                airframe,
                elem_a="tail_boom",
                elem_b=fin_elem,
                contact_tol=0.045,
                name="tail_boom_reaches_tail_fin_support",
            )
            ctx.expect_contact(
                airframe,
                airframe,
                elem_a=fin_elem,
                elem_b="tail_gearbox",
                contact_tol=0.045,
                name="tail_fin_support_carries_tail_gearbox",
            )
    ctx.check_model_valid()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.120)
    ctx.fail_if_joint_mating_has_gap()
    if "main_rotor" in names:
        main_rotor = object_model.get_part("main_rotor")
        main_visuals = {v.name for v in main_rotor.visuals}
        ctx.check(
            "main_rotor_has_hub_and_control_links",
            {"mast", "swashplate", "hub", "hub_cap", "lead_lag_damper_0"}.issubset(main_visuals),
        )
        for idx in range(r.main_blade_count):
            ctx.expect_contact(
                main_rotor,
                main_rotor,
                elem_a="hub",
                elem_b=f"cuff_{idx}",
                contact_tol=0.015,
                name=f"main_rotor_cuff_{idx}_captured_by_hub",
            )
            ctx.expect_contact(
                main_rotor,
                main_rotor,
                elem_a=f"cuff_{idx}",
                elem_b=f"main_blade_{idx}",
                contact_tol=0.015,
                name=f"main_blade_{idx}_root_seated_in_cuff",
            )
            ctx.expect_contact(
                main_rotor,
                main_rotor,
                elem_a="swashplate",
                elem_b=f"pitch_link_{idx}",
                contact_tol=0.180,
                name=f"pitch_link_{idx}_reaches_swashplate",
            )
    if "tail_rotor" in names:
        tail_rotor = object_model.get_part("tail_rotor")
        tail_visuals = {v.name for v in tail_rotor.visuals}
        ctx.check(
            "tail_rotor_has_hub_shaft_and_cuffs",
            {"hub", "input_shaft", "hub_block", "tail_cuff_0"}.issubset(tail_visuals),
        )
        for idx in range(r.tail_blade_count):
            ctx.expect_contact(
                tail_rotor,
                tail_rotor,
                elem_a="hub_block",
                elem_b=f"tail_cuff_{idx}",
                contact_tol=0.020,
                name=f"tail_cuff_{idx}_captured_by_hub",
            )
            ctx.expect_contact(
                tail_rotor,
                tail_rotor,
                elem_a=f"tail_cuff_{idx}",
                elem_b=f"tail_blade_{idx}",
                contact_tol=0.020,
                name=f"tail_blade_{idx}_root_seated_in_cuff",
            )
    ctx.check(
        "main_blade_count",
        len(
            [
                v
                for v in object_model.get_part("main_rotor").visuals
                if v.name.startswith("main_blade_")
            ]
        )
        == r.main_blade_count,
    )
    ctx.check(
        "tail_blade_count",
        len(
            [
                v
                for v in object_model.get_part("tail_rotor").visuals
                if v.name.startswith("tail_blade_")
            ]
        )
        == r.tail_blade_count,
    )
    if r.doors_service_module == "none":
        ctx.check("no_door_joints", not any("door" in name for name in joints))
    return ctx.report()


HelicopterConfig = SingleRotorHelicopterConfig
ResolvedHelicopterConfig = ResolvedSingleRotorHelicopterConfig

__all__ = [
    "SingleRotorHelicopterConfig",
    "ResolvedSingleRotorHelicopterConfig",
    "HelicopterConfig",
    "ResolvedHelicopterConfig",
    "config_from_seed",
    "resolve_config",
    "build_single_rotor_helicopter",
    "build_seeded_single_rotor_helicopter",
    "slot_choices_for_seed",
    "run_single_rotor_helicopter_tests",
]
