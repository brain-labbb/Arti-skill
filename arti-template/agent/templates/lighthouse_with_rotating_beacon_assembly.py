"""Lighthouse with rotating beacon assembly — modular procedural template.

A grounded static tower (Slot A) carrying a gallery + lantern + roof stack
(Slot B), with exactly one CONTINUOUS-rotating beacon optic (Slot C) about
the vertical Z axis. Slot D selects the beacon's bearing topology — beacon
parented directly to the body, or to a separate FIXED pedestal/shaft. Slot E
gates an optional second REVOLUTE access opening (door / gate / hatch / trap).

Identity invariant: every seed produces exactly one CONTINUOUS joint with
axis (0,0,±1) joining the tower (or pedestal/shaft) to the beacon.

Topology distinct combos (≥5 required for module_topology_diversity gate):
  Slot B (3) × Slot D (3) × Slot E (5) = 45 distinct (slot, module) tuples,
  multiplied by Slot A (4) × Slot C (4) for geometric variety.

seed=0 picks anchors: lathe_masonry_shell + baked_into_body +
stacked_optic_ring_cage + direct_to_body + none.
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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

__modular__ = True

# Baked gallery-deck slab thickness (see `_bake_lantern_visuals`). The deck sits
# on top of the tower shaft (z in [gallery_z, gallery_z + this]); the shaft-bearing
# FIXED weld rests the central shaft on its top surface. Single source (Contract 3c)
# so the weld interface tracks the deck instead of a fixed offset below the optic.
_BAKED_GALLERY_DECK_H = 0.12


TowerBodyStyle = Literal[
    "lathe_masonry_shell",
    "mesh_prism_polygonal",
    "cadquery_union_shell",
    "skeletal_lattice_truss",
]
LanternHousingStyle = Literal[
    "baked_into_body",
    "separate_lantern_stack",
    "full_fixed_stack",
]
BeaconOpticStyle = Literal[
    "stacked_optic_ring_cage",
    "searchlight_reflector_armlamp",
    "drum_fresnel_omni",
    "bivalve_birdcage",
]
BeaconBearingTopology = Literal[
    "direct_to_body",
    "separate_fixed_pedestal_part",
    "separate_fixed_shaft_part",
]
AccessOpening = Literal[
    "none",
    "lantern_wall_door",
    "gallery_rail_gate",
    "tower_hatch",
    "gallery_trap_door",
]
PaletteTheme = Literal["white_red", "gray_stone", "iron_lattice", "ivory_seaside"]


TOWER_BODY_MODULES: tuple[TowerBodyStyle, ...] = (
    "lathe_masonry_shell",
    "mesh_prism_polygonal",
    "cadquery_union_shell",
    "skeletal_lattice_truss",
)
LANTERN_HOUSING_MODULES: tuple[LanternHousingStyle, ...] = (
    "baked_into_body",
    "separate_lantern_stack",
    "full_fixed_stack",
)
BEACON_OPTIC_MODULES: tuple[BeaconOpticStyle, ...] = (
    "stacked_optic_ring_cage",
    "searchlight_reflector_armlamp",
    "drum_fresnel_omni",
    "bivalve_birdcage",
)
BEACON_BEARING_MODULES: tuple[BeaconBearingTopology, ...] = (
    "direct_to_body",
    "separate_fixed_pedestal_part",
    "separate_fixed_shaft_part",
)
ACCESS_OPENING_MODULES: tuple[AccessOpening, ...] = (
    # NOTE: `lantern_wall_door` was dropped from the RNG pool after a user
    # review — the swinging door's geometric coherence with the lantern's
    # rotating beacon is too noisy to enforce procedurally, so the spec's
    # 5-candidate Slot E now samples 4 candidates and the lantern keeps a
    # full 8-pane glass ring. The Literal type still includes the value for
    # backwards compatibility (callers passing it manually will be rejected
    # by `resolve_config`).
    "none",
    "gallery_rail_gate",
    "tower_hatch",
    "gallery_trap_door",
)

PALETTES: dict[str, dict[str, tuple[float, float, float, float]]] = {
    "white_red": {
        "body": (0.92, 0.92, 0.90, 1.0),
        "accent": (0.78, 0.10, 0.10, 1.0),
        "lantern": (0.30, 0.32, 0.36, 1.0),
        "glass": (0.65, 0.80, 0.92, 1.0),
        "optic": (0.95, 0.95, 0.80, 1.0),
        "steel": (0.55, 0.57, 0.60, 1.0),
        "dark": (0.10, 0.10, 0.12, 1.0),
        "wood": (0.45, 0.30, 0.18, 1.0),
    },
    "gray_stone": {
        "body": (0.55, 0.55, 0.57, 1.0),
        "accent": (0.38, 0.38, 0.40, 1.0),
        "lantern": (0.28, 0.28, 0.30, 1.0),
        "glass": (0.60, 0.72, 0.85, 1.0),
        "optic": (0.92, 0.92, 0.78, 1.0),
        "steel": (0.48, 0.50, 0.54, 1.0),
        "dark": (0.10, 0.10, 0.12, 1.0),
        "wood": (0.40, 0.28, 0.18, 1.0),
    },
    "iron_lattice": {
        "body": (0.22, 0.24, 0.26, 1.0),
        "accent": (0.40, 0.10, 0.08, 1.0),
        "lantern": (0.28, 0.28, 0.30, 1.0),
        "glass": (0.55, 0.72, 0.88, 1.0),
        "optic": (0.94, 0.92, 0.78, 1.0),
        "steel": (0.55, 0.55, 0.58, 1.0),
        "dark": (0.07, 0.07, 0.09, 1.0),
        "wood": (0.42, 0.28, 0.18, 1.0),
    },
    "ivory_seaside": {
        "body": (0.96, 0.94, 0.86, 1.0),
        "accent": (0.20, 0.32, 0.48, 1.0),
        "lantern": (0.32, 0.34, 0.40, 1.0),
        "glass": (0.62, 0.80, 0.92, 1.0),
        "optic": (0.96, 0.94, 0.82, 1.0),
        "steel": (0.58, 0.60, 0.62, 1.0),
        "dark": (0.10, 0.10, 0.12, 1.0),
        "wood": (0.45, 0.32, 0.18, 1.0),
    },
}


@dataclass(frozen=True)
class LighthouseWithRotatingBeaconAssemblyConfig:
    tower_body_style: TowerBodyStyle | None = None
    lantern_housing_style: LanternHousingStyle | None = None
    beacon_optic_style: BeaconOpticStyle | None = None
    beacon_bearing_topology: BeaconBearingTopology | None = None
    access_opening: AccessOpening | None = None
    palette_theme: PaletteTheme = "white_red"
    tower_height: float = 6.0
    tower_base_radius: float = 1.6
    tower_top_radius: float = 1.05
    lantern_radius: float = 1.10
    lantern_height: float = 0.95
    roof_height: float = 0.65
    door_open_upper: float = 1.35
    beacon_axis_sign: int = 1  # +1 or -1
    palette: dict[str, tuple[float, float, float, float]] = field(
        default_factory=lambda: dict(PALETTES["white_red"])
    )


@dataclass(frozen=True)
class ResolvedLighthouseWithRotatingBeaconAssemblyConfig:
    tower_body_style: TowerBodyStyle
    lantern_housing_style: LanternHousingStyle
    beacon_optic_style: BeaconOpticStyle
    beacon_bearing_topology: BeaconBearingTopology
    access_opening: AccessOpening
    palette_theme: PaletteTheme
    tower_height: float
    tower_base_radius: float
    tower_top_radius: float
    lantern_radius: float
    lantern_height: float
    roof_height: float
    door_open_upper: float
    beacon_axis_sign: int
    # Derived heights:
    gallery_z: float  # top-of-tower-shaft (gallery deck level)
    lantern_z: float  # top of gallery / bottom of lantern_room
    lantern_top_z: float  # top of lantern_room
    roof_top_z: float
    beacon_base_z: float  # z where beacon sits (bottom of optic)
    beacon_top_z: float
    palette: dict[str, tuple[float, float, float, float]]


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(v)))


def config_from_seed(seed: int) -> LighthouseWithRotatingBeaconAssemblyConfig:
    if seed == 0:
        return LighthouseWithRotatingBeaconAssemblyConfig(
            tower_body_style="lathe_masonry_shell",
            lantern_housing_style="baked_into_body",
            beacon_optic_style="stacked_optic_ring_cage",
            beacon_bearing_topology="direct_to_body",
            access_opening="none",
            palette_theme="white_red",
            tower_height=6.0,
            tower_base_radius=1.6,
            tower_top_radius=1.05,
            lantern_radius=1.10,
            lantern_height=0.95,
            roof_height=0.65,
            door_open_upper=1.35,
            beacon_axis_sign=1,
        )
    rng = random.Random(seed)
    base_r = rng.uniform(0.55, 2.6)
    top_r = base_r * rng.uniform(0.55, 0.85)
    height = rng.uniform(2.5, 12.0)
    lantern_r = max(top_r * 0.85, rng.uniform(0.45, 1.5))
    return LighthouseWithRotatingBeaconAssemblyConfig(
        tower_body_style=rng.choice(TOWER_BODY_MODULES),  # type: ignore[arg-type]
        lantern_housing_style=rng.choice(LANTERN_HOUSING_MODULES),  # type: ignore[arg-type]
        beacon_optic_style=rng.choice(BEACON_OPTIC_MODULES),  # type: ignore[arg-type]
        beacon_bearing_topology=rng.choice(BEACON_BEARING_MODULES),  # type: ignore[arg-type]
        access_opening=rng.choice(ACCESS_OPENING_MODULES),  # type: ignore[arg-type]
        palette_theme=rng.choice(tuple(PALETTES.keys())),  # type: ignore[arg-type]
        tower_height=round(height, 3),
        tower_base_radius=round(base_r, 3),
        tower_top_radius=round(top_r, 3),
        lantern_radius=round(lantern_r, 3),
        lantern_height=round(rng.uniform(0.55, 1.35), 3),
        roof_height=round(rng.uniform(0.40, 0.95), 3),
        door_open_upper=round(rng.uniform(1.05, 1.90), 3),
        beacon_axis_sign=rng.choice((1, -1)),
    )


def resolve_config(
    config: LighthouseWithRotatingBeaconAssemblyConfig,
) -> ResolvedLighthouseWithRotatingBeaconAssemblyConfig:
    tower_body = config.tower_body_style or "lathe_masonry_shell"
    lantern_housing = config.lantern_housing_style or "baked_into_body"
    beacon_optic = config.beacon_optic_style or "stacked_optic_ring_cage"
    bearing = config.beacon_bearing_topology or "direct_to_body"
    access = config.access_opening or "none"
    for value, pool, label in (
        (tower_body, TOWER_BODY_MODULES, "tower_body_style"),
        (lantern_housing, LANTERN_HOUSING_MODULES, "lantern_housing_style"),
        (beacon_optic, BEACON_OPTIC_MODULES, "beacon_optic_style"),
        (bearing, BEACON_BEARING_MODULES, "beacon_bearing_topology"),
        (access, ACCESS_OPENING_MODULES, "access_opening"),
    ):
        if value not in pool:
            raise ValueError(f"Unsupported {label}: {value!r}")
    if config.palette_theme not in PALETTES:
        raise ValueError(f"Unsupported palette_theme: {config.palette_theme!r}")

    tower_height = _clamp(config.tower_height, 1.5, 16.0)
    base_r = _clamp(config.tower_base_radius, 0.40, 3.0)
    top_r = _clamp(config.tower_top_radius, 0.30, base_r * 0.95)
    lantern_r = _clamp(config.lantern_radius, max(0.30, top_r * 0.7), top_r * 1.20)
    lantern_h = _clamp(config.lantern_height, 0.45, 1.6)
    roof_h = _clamp(config.roof_height, 0.30, 1.1)

    gallery_z = tower_height
    lantern_z = gallery_z + 0.10  # gallery deck thickness
    lantern_top_z = lantern_z + lantern_h
    roof_top_z = lantern_top_z + roof_h
    beacon_base_z = lantern_z + 0.10
    beacon_top_z = lantern_top_z - 0.08

    return ResolvedLighthouseWithRotatingBeaconAssemblyConfig(
        tower_body_style=tower_body,
        lantern_housing_style=lantern_housing,
        beacon_optic_style=beacon_optic,
        beacon_bearing_topology=bearing,
        access_opening=access,
        palette_theme=config.palette_theme,
        tower_height=tower_height,
        tower_base_radius=base_r,
        tower_top_radius=top_r,
        lantern_radius=lantern_r,
        lantern_height=lantern_h,
        roof_height=roof_h,
        door_open_upper=_clamp(config.door_open_upper, 1.00, 1.95),
        beacon_axis_sign=1 if config.beacon_axis_sign >= 0 else -1,
        gallery_z=gallery_z,
        lantern_z=lantern_z,
        lantern_top_z=lantern_top_z,
        roof_top_z=roof_top_z,
        beacon_base_z=beacon_base_z,
        beacon_top_z=beacon_top_z,
        palette=dict(PALETTES[config.palette_theme]),
    )


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #


def _tapered_cyl(
    *,
    name: str,
    radius_bot: float,
    radius_top: float,
    length: float,
    material: str,
    segments: int = 24,
):
    """Approximate a tapered shell using a LatheGeometry profile.

    Profile points are (r, z) tuples around the rotation axis (the part-local
    z axis). The resulting visual is a closed solid cone-frustum-like shape
    with the bottom face at z=0 and top face at z=length.
    """
    profile = [
        (0.0, 0.0),
        (radius_bot, 0.0),
        (radius_top, length),
        (0.0, length),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=segments), name)


# --------------------------------------------------------------------------- #
# Slot A : tower body
# --------------------------------------------------------------------------- #


def _doorway_geometry(
    r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig,
) -> tuple[float, float, float, float]:
    """Return (door_top_z, door_w, door_h, door_floor_z) for the entrance.

    Coordinates are in tower frame.

    * ``door_floor_z`` = top of the plinth (the door's bottom sits here so
      the swinging panel does NOT punch through the plinth slab).
    * ``door_top_z`` = top of the doorway frame (just below the lintel).
    * ``door_h`` = door panel height, sized to fit the gap between the
      plinth top and the header bottom inside the doorway frame.
    * A header strip (z from ``door_floor_z + door_h`` to ``door_top_z``)
      closes the +x face above the door so the wall reads as continuous.
    """
    door_top_z = max(0.40, min(r.tower_height * 0.18, 1.5))
    door_w = max(0.30, min(r.tower_base_radius * 1.10, 0.90))
    plinth_h = max(0.18, 0.05 * r.tower_height)
    door_floor_z = plinth_h
    available_h = max(0.20, door_top_z - door_floor_z)
    header_h = max(0.08, min(available_h * 0.18, 0.20))
    door_h = max(0.15, available_h - header_h)
    return door_top_z, door_w, door_h, door_floor_z


def _bake_doorway_frame(
    part, r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig
) -> tuple[float, float, float, float]:
    """Bake a 4-walled entrance room + lintel at the tower base, with a
    door-sized opening centered on the +x face that sits ABOVE the plinth.

    The door opening (a hole in the +x face) spans
    y ∈ [-door_w/2, door_w/2] and z ∈ [door_floor_z, door_floor_z+door_h].
    Everywhere else on the +x face — both flanks (jambs) and above the
    door (header) — is solid wall, so the entrance reads as a normal house
    wall with a door cut into the middle. Returns
    (door_top_z, door_w, door_h, door_floor_z).
    """
    R = r.tower_base_radius
    door_top_z, door_w, door_h, door_floor_z = _doorway_geometry(r)
    wall_t = max(0.08, R * 0.10)
    # Rear wall on -x side (full width along y)
    part.visual(
        Box((wall_t, R * 1.80, door_top_z)),
        origin=Origin(xyz=(-R * 0.90, 0.0, door_top_z * 0.5)),
        material="body",
        name="entry_rear_wall",
    )
    # Left wall on -y side (spans -x to +x)
    part.visual(
        Box((R * 1.80, wall_t, door_top_z)),
        origin=Origin(xyz=(0.0, -R * 0.90, door_top_z * 0.5)),
        material="body",
        name="entry_left_wall",
    )
    # Right wall on +y side
    part.visual(
        Box((R * 1.80, wall_t, door_top_z)),
        origin=Origin(xyz=(0.0, +R * 0.90, door_top_z * 0.5)),
        material="body",
        name="entry_right_wall",
    )
    # Front wall on +x side — split into two jamb panels flanking the
    # door-sized opening so the entrance looks like a normal wall with a
    # door cut into the middle.
    front_wall_inner_y = R * 0.90 - wall_t * 0.5
    jamb_inner_y = door_w * 0.5
    jamb_span = front_wall_inner_y - jamb_inner_y
    if jamb_span > 0.04:
        jamb_center_y = (front_wall_inner_y + jamb_inner_y) * 0.5
        part.visual(
            Box((wall_t, jamb_span, door_top_z)),
            origin=Origin(xyz=(R * 0.90, -jamb_center_y, door_top_z * 0.5)),
            material="body",
            name="entry_front_left_jamb",
        )
        part.visual(
            Box((wall_t, jamb_span, door_top_z)),
            origin=Origin(xyz=(R * 0.90, +jamb_center_y, door_top_z * 0.5)),
            material="body",
            name="entry_front_right_jamb",
        )
    # Header above the door opening — sits between (door_floor_z + door_h)
    # and door_top_z. Closes the +x face above the door so the wall reads
    # as continuous everywhere except the rectangular door hole.
    door_top_open_z = door_floor_z + door_h
    header_h = max(0.04, door_top_z - door_top_open_z)
    if header_h > 0.04:
        part.visual(
            Box((wall_t, door_w, header_h)),
            origin=Origin(
                xyz=(R * 0.90, 0.0, door_top_open_z + header_h * 0.5),
            ),
            material="body",
            name="entry_front_header",
        )
    # Lintel above door — flat slab covering the doorway opening, AABB-
    # overlaps the upper shaft so the part stays a single island.
    lintel_h = 0.15
    part.visual(
        Box((R * 1.80, R * 1.80, lintel_h)),
        origin=Origin(xyz=(0.0, 0.0, door_top_z + lintel_h * 0.5)),
        material="accent",
        name="entry_lintel",
    )
    return door_top_z, door_w, door_h, door_floor_z


def _build_tower_body(part, r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig) -> None:
    """Emit the tower shaft visuals (lathe / mesh-prism / cadquery / lattice).

    All variants emit a connected shaft running from z=0 to z=tower_height,
    with a square plinth at the base. To ensure single-island connectivity
    and to provide a predictable host for baked door anchors, every style
    also emits a central full-height core Cylinder (radius R_top*0.45) that
    runs from the plinth top to z=tower_height.

    When ``access_opening == "tower_hatch"``, the tower's bottom segment
    (z=[0, door_top_z]) is replaced by a U-shaped doorway frame (3 walls +
    lintel) so the hatch actually swings into a real opening rather than
    against a solid wall. The upper shaft and tower_core both start above
    the lintel.

    Every visual must AABB-overlap with at least one neighbor so the part
    stays a single connected island.
    """
    style = r.tower_body_style
    H = r.tower_height
    R_bot = r.tower_base_radius
    R_top = r.tower_top_radius
    plinth_h = max(0.18, 0.05 * H)
    has_hatch = r.access_opening == "tower_hatch"
    if has_hatch:
        door_top_z, _, _, _ = _bake_doorway_frame(part, r)
        # Upper segments start ABOVE the lintel.
        shaft_floor_z = door_top_z + 0.15
    else:
        shaft_floor_z = plinth_h * 0.5

    # plinth — a wide low Box that the shaft sits on. Always present, so
    # the tower has a real footprint regardless of whether the doorway
    # frame is baked.
    plinth_size = max(R_bot * 2.1, 0.6)
    part.visual(
        Box((plinth_size, plinth_size, plinth_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h * 0.5)),
        material="accent",
        name="plinth",
    )

    # Central core column — anchor for non-base doors (lantern / gallery
    # gates / trap doors). Starts above the doorway lintel when tower_hatch
    # is gated so it doesn't fill the opening.
    core_r = max(0.10, R_top * 0.45)
    core_bot_z = shaft_floor_z
    core_top_z = H
    if core_top_z > core_bot_z + 0.05:
        part.visual(
            Cylinder(radius=core_r, length=core_top_z - core_bot_z),
            origin=Origin(xyz=(0.0, 0.0, (core_bot_z + core_top_z) * 0.5)),
            material="steel",
            name="tower_core",
        )

    if style == "lathe_masonry_shell":
        # LatheGeometry tapered shell from shaft_floor_z (above plinth or
        # above the doorway lintel) to z=H.
        shaft_bot_z = max(plinth_h * 0.7, shaft_floor_z - 0.02)
        shaft_length = max(0.10, H - shaft_bot_z)
        part.visual(
            _tapered_cyl(
                name="lathe_shaft",
                radius_bot=R_bot * 0.95,
                radius_top=R_top * 1.0,
                length=shaft_length,
                material="body",
            ),
            origin=Origin(xyz=(0.0, 0.0, shaft_bot_z)),
            material="body",
            name="lathe_shaft",
        )
        # Decorative cornice ring just below the gallery (red painted band)
        part.visual(
            Cylinder(radius=R_top * 1.08, length=H * 0.04),
            origin=Origin(xyz=(0.0, 0.0, H - H * 0.05)),
            material="accent",
            name="cornice_ring",
        )
        # Narrow window slit on the +x side — baked Box visual. The lathe shell
        # is hollow, so the slit must reach inward to the solid tower_core to
        # stay connected (its inner face embeds ~2cm into the core).
        _win_in = core_r - 0.02
        _win_out = R_top * 0.80
        part.visual(
            Box((_win_out - _win_in, R_bot * 0.40, H * 0.06)),
            origin=Origin(xyz=((_win_in + _win_out) * 0.5, 0.0, H * 0.45)),
            material="dark",
            name="window_slit",
        )
    elif style == "mesh_prism_polygonal":
        # Stacked prism segments — approximate via 3 stacked Box columns.
        # Start above the doorway lintel when tower_hatch is gated.
        seg_count = 3
        seg_base_z = shaft_floor_z
        seg_h = max(0.10, (H - seg_base_z) / seg_count)
        for i in range(seg_count):
            t = i / max(1, seg_count - 1) if seg_count > 1 else 0.0
            r_here = R_bot * (1 - t) + R_top * t
            seg_size = r_here * 1.7
            z_bot = seg_base_z + i * seg_h
            part.visual(
                Box((seg_size, seg_size, seg_h)),
                origin=Origin(xyz=(0.0, 0.0, z_bot + seg_h * 0.5)),
                material="body",
                name=f"prism_seg_{i}",
            )
        # Mid-band accent stripe
        part.visual(
            Box((R_top * 1.7, R_top * 1.7, H * 0.04)),
            origin=Origin(xyz=(0.0, 0.0, H * 0.50)),
            material="accent",
            name="prism_stripe",
        )
    elif style == "cadquery_union_shell":
        # Use a stack of frusta as primitive Cylinders unioned visually.
        # Start above the doorway lintel when tower_hatch is gated.
        seg_count = 4
        seg_base_z = shaft_floor_z
        seg_h = max(0.10, (H - seg_base_z) / seg_count)
        for i in range(seg_count):
            t = i / max(1, seg_count - 1) if seg_count > 1 else 0.0
            r_here = R_bot * (1 - t) + R_top * t
            z_bot = seg_base_z + i * seg_h
            part.visual(
                Cylinder(radius=r_here, length=seg_h * 1.05),
                origin=Origin(xyz=(0.0, 0.0, z_bot + seg_h * 0.5)),
                material="body",
                name=f"union_seg_{i}",
            )
        # Decorative shoulder collar near top
        part.visual(
            Cylinder(radius=R_top * 1.10, length=H * 0.04),
            origin=Origin(xyz=(0.0, 0.0, H * 0.78)),
            material="accent",
            name="shoulder_collar",
        )
    else:  # skeletal_lattice_truss
        # 4 vertical Box legs (tapered position) + horizontal Box ring members.
        # When tower_hatch is gated, legs start above the doorway lintel so
        # they don't fill the opening.
        leg_size = max(0.06, R_bot * 0.10)
        ring_levels = 4
        leg_z = shaft_floor_z
        leg_len = max(0.10, H - leg_z)
        for i in range(4):
            ang = i * math.pi / 2.0 + math.pi / 4.0
            # taper inward — pick a midpoint radius
            r_mid = (R_bot + R_top) * 0.5
            x = math.cos(ang) * r_mid
            y = math.sin(ang) * r_mid
            part.visual(
                Box((leg_size, leg_size, leg_len)),
                origin=Origin(xyz=(x, y, leg_z + leg_len * 0.5)),
                material="body",
                name=f"truss_leg_{i}",
            )
        # Horizontal X-braces at intermediate levels — long Box pieces spanning
        # opposite legs so each leg AABB-overlaps with brace AABB.
        for lvl in range(ring_levels):
            t = (lvl + 1) / (ring_levels + 1)
            r_here = R_bot * (1 - t) + R_top * t
            z_here = leg_z + leg_len * t
            ring_len = r_here * 2.4
            # X bar
            part.visual(
                Box((ring_len, leg_size, leg_size)),
                origin=Origin(xyz=(0.0, 0.0, z_here)),
                material="body",
                name=f"truss_ringx_{lvl}",
            )
            # Y bar
            part.visual(
                Box((leg_size, ring_len, leg_size)),
                origin=Origin(xyz=(0.0, 0.0, z_here)),
                material="body",
                name=f"truss_ringy_{lvl}",
            )
        # (Central core is already emitted at the top of _build_tower_body
        # as `tower_core`. The lattice variant just adds the outer truss.)


def _emit_lantern_panes_and_mullions(
    part,
    r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig,
    *,
    name_prefix: str,
    bottom_z: float,
) -> None:
    """Emit the lantern enclosure as 8 mullion posts + 8 glass panes.

    All 8 panes are always emitted (the previous `lantern_wall_door` skip
    logic has been removed — see the comment on ACCESS_OPENING_MODULES).

    Pane Boxes are sized so each pane's rotated AABB intersects both
    adjacent mullion AABBs, keeping the lantern a single connected island.
    """
    LR = r.lantern_radius
    Lh = r.lantern_height
    n = 8
    mullion_size = max(0.06, LR * 0.10)
    mullion_radius = LR * 1.02
    for i in range(n):
        ang = i * math.pi / 4.0
        part.visual(
            Box((mullion_size, mullion_size, Lh)),
            origin=Origin(
                xyz=(
                    math.cos(ang) * mullion_radius,
                    math.sin(ang) * mullion_radius,
                    bottom_z + Lh * 0.5,
                ),
            ),
            material="lantern",
            name=f"{name_prefix}_mullion_{i}",
        )
    pane_chord = 2.0 * LR * math.sin(math.pi / n) * 1.20  # slightly wider than chord
    pane_radial = 0.05
    for i in range(n):
        center_ang = (i + 0.5) * math.pi / 4.0
        cx = LR * math.cos(center_ang)
        cy = LR * math.sin(center_ang)
        part.visual(
            Box((pane_radial, pane_chord, Lh * 0.96)),
            origin=Origin(
                xyz=(cx, cy, bottom_z + Lh * 0.5),
                rpy=(0.0, 0.0, center_ang),
            ),
            material="glass",
            name=f"{name_prefix}_pane_{i}",
        )


def _bake_lantern_visuals(part, r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig) -> None:
    """Slot B = baked_into_body : gallery + lantern + roof as named visuals."""
    LR = r.lantern_radius
    Lh = r.lantern_height
    Rh = r.roof_height
    gz = r.gallery_z
    lz = r.lantern_z
    ltop = r.lantern_top_z
    rtop = r.roof_top_z

    # Gallery deck — a thick disk slightly wider than lantern
    part.visual(
        Cylinder(radius=LR * 1.40, length=_BAKED_GALLERY_DECK_H),
        origin=Origin(xyz=(0.0, 0.0, gz + _BAKED_GALLERY_DECK_H * 0.5)),
        material="accent",
        name="gallery_deck",
    )
    # Gallery rail — annular slab visualized as a thin tall ring (TorusGeometry)
    part.visual(
        mesh_from_geometry(
            TorusGeometry(radius=LR * 1.30, tube=0.04, tubular_segments=32),
            "gallery_rail",
        ),
        origin=Origin(xyz=(0.0, 0.0, gz + 0.30)),
        material="steel",
        name="gallery_rail",
    )
    # Balusters tying the floating rail torus down to the gallery_deck (the
    # baked variant, unlike _emit_gallery_part, otherwise leaves the rail as an
    # island 0.30m above the deck).
    for _i in range(8):
        _ang = _i * math.pi / 4.0
        part.visual(
            Box((0.05, 0.05, 0.36)),
            origin=Origin(xyz=(math.cos(_ang) * LR * 1.30, math.sin(_ang) * LR * 1.30, gz + 0.18)),
            material="steel",
            name=f"gallery_baluster_{_i}",
        )
    # Mullions + glass panes — full 8-pane ring (lantern_wall_door dropped).
    _emit_lantern_panes_and_mullions(
        part,
        r,
        name_prefix="lantern",
        bottom_z=lz,
    )
    # Lantern ceiling disc — wide enough to overlap mullion top and host
    # the roof_cap stack joint origin.
    part.visual(
        Cylinder(radius=LR * 1.10, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, ltop - 0.04)),
        material="lantern",
        name="lantern_ceiling",
    )
    # Roof cap — short cone-like cylinder
    part.visual(
        Cylinder(radius=LR * 1.02, length=Rh),
        origin=Origin(xyz=(0.0, 0.0, ltop + Rh * 0.5)),
        material="accent",
        name="roof_cap",
    )
    # Roof spike / vent
    part.visual(
        Cylinder(radius=LR * 0.07, length=Rh * 0.45),
        origin=Origin(xyz=(0.0, 0.0, rtop + Rh * 0.18)),
        material="dark",
        name="roof_spike",
    )
    # Baked central beacon spindle so beacon (direct_to_body) has real geometry.
    # Extend its base down to the gallery_deck so it is not a floating column.
    _spindle_top = lz + Lh * 0.50
    _spindle_bot = gz
    part.visual(
        Cylinder(radius=max(0.05, LR * 0.10), length=_spindle_top - _spindle_bot),
        origin=Origin(xyz=(0.0, 0.0, (_spindle_top + _spindle_bot) * 0.5)),
        material="steel",
        name="baked_beacon_spindle",
    )


def _bake_minimal_lantern_anchor(
    part, r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig
) -> None:
    """For Slot B != baked, the tower still needs SOME geometry at z=gallery_z
    so the lantern-stack FIXED joint origin lies on real geometry. Emit a thin
    gallery deck plate baked into the tower."""
    LR = r.lantern_radius
    gz = r.gallery_z
    part.visual(
        Cylinder(radius=LR * 1.40, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, gz + 0.10)),
        material="accent",
        name="gallery_deck",
    )


def _bake_door_anchor_visuals(part, r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig) -> None:
    """Door anchors are now baked per-variant:
      * ``tower_hatch`` → `_bake_doorway_frame` (U-walls + lintel at base)
      * ``lantern_wall_door`` → carved out by the lantern pane logic
      * ``gallery_rail_gate`` / ``gallery_trap_door`` → gallery_deck (thick
        Cylinder) provides a sufficient host surface
    This function is kept for forward compatibility / future anchor visuals.
    """
    return None


# --------------------------------------------------------------------------- #
# Slot B helpers : independent lantern-stack parts
# --------------------------------------------------------------------------- #


def _emit_lantern_room_part(
    model: ArticulatedObject,
    r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig,
    *,
    name: str = "lantern_room",
) -> None:
    """Emit a lantern_room part whose part frame origin is at the BOTTOM
    center (so the FIXED joint origin coincides with the part origin and
    sits on real geometry). Glass is built from the full 8-pane ring (the
    earlier `lantern_wall_door` skip variant has been removed)."""
    LR = r.lantern_radius
    Lh = r.lantern_height
    p = model.part(name)
    # Sill ring (low collar) — bottom 10cm. Connects all mullions+panes.
    p.visual(
        Cylinder(radius=LR * 1.02, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="lantern",
        name=f"{name}_sill",
    )
    _emit_lantern_panes_and_mullions(
        p,
        r,
        name_prefix=name,
        bottom_z=0.0,
    )
    # Lantern ceiling — wide disc at the top of the lantern_room, hosts
    # the `lantern_room_to_roof_cap` FIXED joint origin (at z=Lh in part frame).
    p.visual(
        Cylinder(radius=LR * 1.10, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, Lh - 0.04)),
        material="lantern",
        name=f"{name}_ceiling",
    )
    # Beacon-supporting spindle baked in lantern_room. Base extended down into
    # the sill ring (spans 0..0.10) so it is not a floating column.
    _sp_top = Lh * 0.50
    _sp_bot = 0.05
    p.visual(
        Cylinder(radius=max(0.05, LR * 0.10), length=_sp_top - _sp_bot),
        origin=Origin(xyz=(0.0, 0.0, (_sp_top + _sp_bot) * 0.5)),
        material="steel",
        name=f"{name}_spindle",
    )
    p.inertial = Inertial.from_geometry(
        Cylinder(radius=LR, length=Lh), mass=0.5, origin=Origin(xyz=(0.0, 0.0, Lh * 0.5))
    )


def _emit_roof_cap_part(
    model: ArticulatedObject,
    r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig,
) -> None:
    LR = r.lantern_radius
    Rh = r.roof_height
    p = model.part("roof_cap")
    p.visual(
        Cylinder(radius=LR * 1.02, length=Rh),
        origin=Origin(xyz=(0.0, 0.0, Rh * 0.5)),
        material="accent",
        name="roof_cap_disk",
    )
    p.visual(
        Cylinder(radius=LR * 0.07, length=Rh * 0.45),
        origin=Origin(xyz=(0.0, 0.0, Rh + Rh * 0.18)),
        material="dark",
        name="roof_spike",
    )
    p.inertial = Inertial.from_geometry(
        Cylinder(radius=LR, length=Rh), mass=0.2, origin=Origin(xyz=(0.0, 0.0, Rh * 0.5))
    )


def _emit_gallery_part(
    model: ArticulatedObject,
    r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig,
) -> None:
    LR = r.lantern_radius
    p = model.part("gallery")
    # Gallery deck centered on origin
    p.visual(
        Cylinder(radius=LR * 1.40, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material="accent",
        name="gallery_deck",
    )
    p.visual(
        mesh_from_geometry(
            TorusGeometry(radius=LR * 1.30, tube=0.04, tubular_segments=32),
            "gallery_rail",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material="steel",
        name="gallery_rail",
    )
    # Posts to give the deck-rail single-island connectivity (Torus AABB
    # may not overlap the deck Cylinder otherwise).
    for i in range(8):
        ang = i * math.pi / 4.0
        p.visual(
            Box((0.05, 0.05, 0.36)),
            origin=Origin(xyz=(math.cos(ang) * LR * 1.30, math.sin(ang) * LR * 1.30, 0.18)),
            material="steel",
            name=f"gallery_post_{i}",
        )
    p.inertial = Inertial.from_geometry(
        Cylinder(radius=LR * 1.40, length=0.30), mass=0.3, origin=Origin(xyz=(0.0, 0.0, 0.15))
    )


# --------------------------------------------------------------------------- #
# Slot C : beacon optic
# --------------------------------------------------------------------------- #


def _build_beacon_part(
    model: ArticulatedObject,
    r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig,
    *,
    headroom: float,
) -> None:
    """Emit the beacon part with its origin at z=0 (i.e. its part frame
    origin coincides with the rotation joint origin). The optic geometry
    is always vertical (extends in +z from origin).

    ``headroom`` is the vertical room above the joint origin that the
    beacon must fit within, so its top stays below the lantern roof.
    """
    LR = r.lantern_radius
    optic_r = min(LR * 0.70, LR * 0.78)
    # Reserve 10% safety margin below the roof.
    h = max(0.20, min(r.lantern_height * 0.55, headroom * 0.88))
    style = r.beacon_optic_style

    beacon = model.part("beacon")
    # Turntable disc — bottom base
    beacon.visual(
        Cylinder(radius=optic_r * 0.95, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="steel",
        name="turntable",
    )
    # Central lamp column — connects everything vertically
    beacon.visual(
        Cylinder(radius=optic_r * 0.18, length=h),
        origin=Origin(xyz=(0.0, 0.0, h * 0.5 + 0.04)),
        material="steel",
        name="lamp_column",
    )
    # Lamp core (sphere at center) for warm reading
    beacon.visual(
        Sphere(radius=optic_r * 0.16),
        origin=Origin(xyz=(0.0, 0.0, h * 0.55 + 0.04)),
        material="optic",
        name="lamp_core",
    )

    if style == "stacked_optic_ring_cage":
        # Lower + upper optic ring (Torus); plus 6 vertical optic_posts
        for k, frac in enumerate((0.20, 0.85)):
            beacon.visual(
                mesh_from_geometry(
                    TorusGeometry(radius=optic_r * 0.85, tube=0.05, tubular_segments=24),
                    f"optic_ring_{k}",
                ),
                origin=Origin(xyz=(0.0, 0.0, frac * h + 0.04)),
                material="optic",
                name=f"optic_ring_{k}",
            )
        # Posts run from inside the turntable (top at z=0.08) up past both optic
        # rings, tying the outer optic cage to the central turntable/lamp_column
        # stack (otherwise the two are separate islands).
        _post_bot = 0.06
        _post_top = 0.825 * h + 0.04
        for i in range(6):
            ang = i * math.pi / 3.0
            beacon.visual(
                Box((0.05, 0.05, _post_top - _post_bot)),
                origin=Origin(
                    xyz=(
                        math.cos(ang) * optic_r * 0.80,
                        math.sin(ang) * optic_r * 0.80,
                        (_post_bot + _post_top) * 0.5,
                    )
                ),
                material="optic",
                name=f"optic_post_{i}",
            )
        # 4 optic panel glass plates between posts at mid-height
        for i in range(4):
            ang = i * math.pi / 2.0 + math.pi / 4.0
            beacon.visual(
                Box((optic_r * 0.18, optic_r * 0.08, h * 0.55)),
                origin=Origin(
                    xyz=(
                        math.cos(ang) * optic_r * 0.78,
                        math.sin(ang) * optic_r * 0.78,
                        h * 0.50 + 0.04,
                    ),
                    rpy=(0.0, 0.0, ang + math.pi / 2.0),
                ),
                material="glass",
                name=f"optic_panel_{i}",
            )
    elif style == "searchlight_reflector_armlamp":
        # Bearing collar on top of turntable
        beacon.visual(
            Cylinder(radius=optic_r * 0.55, length=0.10),
            origin=Origin(xyz=(0.0, 0.0, 0.12)),
            material="steel",
            name="bearing_collar",
        )
        # Frame arm extending +X — Box
        beacon.visual(
            Box((optic_r * 1.6, optic_r * 0.20, optic_r * 0.20)),
            origin=Origin(xyz=(optic_r * 0.50, 0.0, h * 0.55 + 0.04)),
            material="steel",
            name="frame_arm",
        )
        # Reflector dish - large cylinder front-facing
        beacon.visual(
            Cylinder(
                radius=optic_r * 0.55,
                length=optic_r * 0.30,
            ),
            origin=Origin(
                xyz=(optic_r * 1.05, 0.0, h * 0.55 + 0.04),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material="optic",
            name="reflector_shell",
        )
        # Lamp / bulb in front of reflector
        beacon.visual(
            Sphere(radius=optic_r * 0.18),
            origin=Origin(xyz=(optic_r * 1.20, 0.0, h * 0.55 + 0.04)),
            material="optic",
            name="lamp_bulb",
        )
        # Counterweight on opposite side. Shifted +x so its front face overlaps
        # the frame_arm back end (arm spans x -0.30..1.30 * optic_r); at -0.55 it
        # floated ~0.017m short of the arm.
        beacon.visual(
            Box((optic_r * 0.45, optic_r * 0.35, optic_r * 0.35)),
            origin=Origin(xyz=(-optic_r * 0.46, 0.0, h * 0.55 + 0.04)),
            material="dark",
            name="counterweight",
        )
    elif style == "drum_fresnel_omni":
        # Tall sleeve carriage with Fresnel ring stack
        beacon.visual(
            Cylinder(radius=optic_r * 0.85, length=h * 0.80),
            origin=Origin(xyz=(0.0, 0.0, h * 0.50 + 0.04)),
            material="glass",
            name="carriage_sleeve",
        )
        # 6 Torus Fresnel rings stacked
        for k in range(6):
            z = h * 0.18 + k * h * 0.13
            beacon.visual(
                mesh_from_geometry(
                    TorusGeometry(radius=optic_r * 0.85, tube=0.04, tubular_segments=24),
                    f"fresnel_ring_{k}",
                ),
                origin=Origin(xyz=(0.0, 0.0, z + 0.04)),
                material="optic",
                name=f"fresnel_ring_{k}",
            )
        # Crown ring on top
        beacon.visual(
            Cylinder(radius=optic_r * 0.90, length=0.08),
            origin=Origin(xyz=(0.0, 0.0, h + 0.04)),
            material="accent",
            name="lens_crown_ring",
        )
    else:  # bivalve_birdcage
        # Cross-arm: horizontal Box at mid-height
        beacon.visual(
            Box((optic_r * 1.6, optic_r * 0.22, optic_r * 0.18)),
            origin=Origin(xyz=(0.0, 0.0, h * 0.55 + 0.04)),
            material="steel",
            name="cross_arm",
        )
        # Two lens drums on either end
        for sign, side in ((+1, "right"), (-1, "left")):
            beacon.visual(
                Cylinder(radius=optic_r * 0.45, length=optic_r * 0.30),
                origin=Origin(
                    xyz=(sign * optic_r * 0.80, 0.0, h * 0.55 + 0.04),
                    rpy=(0.0, math.pi * 0.5, 0.0),
                ),
                material="optic",
                name=f"lens_drum_{side}",
            )
            # Birdcage frame around each drum
            beacon.visual(
                mesh_from_geometry(
                    TorusGeometry(radius=optic_r * 0.40, tube=0.04, tubular_segments=18),
                    f"birdcage_ring_{side}",
                ),
                origin=Origin(
                    xyz=(sign * optic_r * 0.80, 0.0, h * 0.55 + 0.04),
                    rpy=(0.0, math.pi * 0.5, 0.0),
                ),
                material="steel",
                name=f"birdcage_ring_{side}",
            )

    beacon.inertial = Inertial.from_geometry(
        Cylinder(radius=optic_r, length=h),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.0, h * 0.5 + 0.04)),
    )


# --------------------------------------------------------------------------- #
# Slot D : beacon bearing topology  (pedestal / shaft FIXED parts)
# --------------------------------------------------------------------------- #


def _emit_pedestal_part(
    model: ArticulatedObject,
    r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig,
) -> None:
    """Independent pedestal part — base flange + column + bearing cap. Part
    frame origin is at the BOTTOM of the base flange."""
    LR = r.lantern_radius
    h_total = max(0.30, r.lantern_height * 0.55)
    p = model.part("pedestal")
    p.visual(
        Cylinder(radius=LR * 0.45, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material="accent",
        name="base_flange",
    )
    p.visual(
        Cylinder(radius=LR * 0.20, length=h_total - 0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06 + (h_total - 0.12) * 0.5)),
        material="steel",
        name="column",
    )
    p.visual(
        Cylinder(radius=LR * 0.35, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, h_total - 0.03)),
        material="accent",
        name="bearing_cap",
    )
    p.inertial = Inertial.from_geometry(
        Cylinder(radius=LR * 0.45, length=h_total),
        mass=0.4,
        origin=Origin(xyz=(0.0, 0.0, h_total * 0.5)),
    )


def _emit_central_shaft_part(
    model: ArticulatedObject,
    r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig,
) -> None:
    """Independent central shaft part — shaft_base + shaft_main. Part frame
    origin at bottom."""
    LR = r.lantern_radius
    h_total = max(0.40, r.lantern_height * 0.80)
    p = model.part("central_shaft")
    p.visual(
        Cylinder(radius=LR * 0.30, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material="accent",
        name="shaft_base",
    )
    p.visual(
        Cylinder(radius=LR * 0.10, length=h_total - 0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.06 + (h_total - 0.06) * 0.5)),
        material="steel",
        name="shaft_main",
    )
    p.inertial = Inertial.from_geometry(
        Cylinder(radius=LR * 0.30, length=h_total),
        mass=0.3,
        origin=Origin(xyz=(0.0, 0.0, h_total * 0.5)),
    )


# --------------------------------------------------------------------------- #
# Slot E : access opening  (single REVOLUTE child part)
# --------------------------------------------------------------------------- #


def _emit_door_part(
    model: ArticulatedObject,
    r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig,
    *,
    kind: AccessOpening,
) -> tuple[str, tuple[float, float, float], tuple[float, float, float], str]:
    """Emit the gated access-opening part. Returns (part_name, joint_origin_xyz,
    joint_axis, parent_part_name).

    The door part is built with its part frame origin AT THE HINGE LINE, so
    the joint origin in the parent's frame is the hinge-line center and the
    door's hinge_barrel sits at the origin (covering 0,0,0 in part-frame).
    """
    LR = r.lantern_radius
    sign = r.beacon_axis_sign
    # `lantern_wall_door` is intentionally NOT handled here — the candidate
    # has been dropped from the RNG pool (see ACCESS_OPENING_MODULES) after a
    # user review; the lantern keeps a full 8-pane glass ring with no swinging
    # door. Calling _emit_door_part(..., kind="lantern_wall_door") will fall
    # through to the ValueError raise at the end of this function.
    if kind == "gallery_rail_gate":
        gate_h = 0.30
        gate_w = min(LR * 0.60, 0.55)
        gate_t = 0.03
        p = model.part("gallery_gate")
        p.visual(
            Cylinder(radius=0.035, length=gate_h),
            origin=Origin(xyz=(0.0, 0.0, gate_h * 0.5)),
            material="steel",
            name="hinge_barrel",
        )
        p.visual(
            Box((gate_w, gate_t, gate_h)),
            origin=Origin(xyz=(gate_w * 0.5 + 0.02, 0.0, gate_h * 0.5)),
            material="steel",
            name="gate_frame",
        )
        # Crossbars to make it look like skeletal gate (single-island via frame)
        for k, frac in enumerate((0.30, 0.55, 0.80)):
            p.visual(
                Box((gate_w * 0.85, gate_t * 0.7, 0.03)),
                origin=Origin(xyz=(gate_w * 0.5 + 0.02, 0.0, gate_h * frac)),
                material="steel",
                name=f"gate_bar_{k}",
            )
        p.inertial = Inertial.from_geometry(
            Box((gate_w, gate_t, gate_h)),
            mass=0.08,
            origin=Origin(xyz=(gate_w * 0.5, 0.0, gate_h * 0.5)),
        )
        # The gallery deck on tower spans z=[gz, gz+0.20] (minimal anchor) or
        # z=[gz, gz+0.12] (baked variant). Place joint origin at z=gz+0.05,
        # well inside the deck.
        return (
            "gallery_gate",
            (LR * 1.25, 0.0, r.gallery_z + 0.05),
            (0.0, 0.0, float(sign)),
            "tower",
        )
    if kind == "tower_hatch":
        # The hatch swings into a real doorway opening at the tower base.
        # See `_bake_doorway_frame` for the wall + jamb layout. Panel spans
        # the full door_w so the closed door visually fills the doorway hole.
        # The bottom of the door is at door_floor_z (top of the plinth) so
        # the swinging panel never punches through the plinth slab.
        door_top_z, door_w, door_h, door_floor_z = _doorway_geometry(r)
        panel_w = door_w - 0.015  # small clearance from jamb sides
        hatch_t = 0.04
        R = r.tower_base_radius
        p = model.part("hatch")
        # Vertical hinge_barrel at part frame origin
        p.visual(
            Cylinder(radius=0.05, length=door_h),
            origin=Origin(xyz=(0.0, 0.0, door_h * 0.5)),
            material="steel",
            name="hinge_barrel",
        )
        # Door panel extends in -y direction (tangentially across the
        # opening). At joint=0 (closed) the panel covers the doorway; at
        # joint=upper the panel rotates outward about +z.
        p.visual(
            Box((hatch_t, panel_w, door_h * 0.95)),
            origin=Origin(xyz=(hatch_t * 0.5 + 0.01, -panel_w * 0.5, door_h * 0.5)),
            material="lantern",
            name="hatch_panel",
        )
        p.visual(
            Box((hatch_t * 0.6, panel_w * 0.55, door_h * 0.4)),
            origin=Origin(xyz=(hatch_t * 0.5 + 0.012, -panel_w * 0.5, door_h * 0.65)),
            material="glass",
            name="hatch_window",
        )
        p.inertial = Inertial.from_geometry(
            Box((hatch_t, panel_w, door_h)),
            mass=0.07,
            origin=Origin(xyz=(0.0, -panel_w * 0.5, door_h * 0.5)),
        )
        # Joint origin: +y jamb of the doorway (right-jamb hinge), z is at
        # the plinth top so the door bottom rests on the plinth slab and
        # never swings into / through it.
        return (
            "hatch",
            (R * 0.85, +door_w * 0.5, door_floor_z),
            (0.0, 0.0, 1.0),
            "tower",
        )
    if kind == "gallery_trap_door":
        leaf_w = min(LR * 0.55, 0.50)
        leaf_d = min(LR * 0.45, 0.40)
        leaf_t = 0.04
        p = model.part("trap_door")
        # Vertical hinge barrel along y axis. Place barrel centered on origin
        # so the part frame origin lies inside the hinge_barrel AABB.
        p.visual(
            Cylinder(radius=0.04, length=leaf_d),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="steel",
            name="hinge_barrel",
        )
        p.visual(
            Box((leaf_w, leaf_d, leaf_t)),
            origin=Origin(xyz=(leaf_w * 0.5 + 0.02, 0.0, 0.0)),
            material="wood",
            name="gate_leaf",
        )
        p.visual(
            Box((leaf_w * 0.15, leaf_d * 0.15, leaf_t * 1.4)),
            origin=Origin(xyz=(leaf_w * 0.8, 0.0, 0.0)),
            material="steel",
            name="latch",
        )
        p.inertial = Inertial.from_geometry(
            Box((leaf_w, leaf_d, leaf_t)),
            mass=0.10,
            origin=Origin(xyz=(leaf_w * 0.5, 0.0, 0.0)),
        )
        # Hinge along y; place joint origin INSIDE the gallery_deck Cylinder
        # (z=[gz, gz+0.12 or 0.20]). z=gz+0.06 is safely inside.
        return (
            "trap_door",
            (LR * 0.75, 0.0, r.gallery_z + 0.06),
            (0.0, float(sign), 0.0),
            "tower",
        )
    raise ValueError(f"Unsupported access_opening: {kind!r}")


# --------------------------------------------------------------------------- #
# Build
# --------------------------------------------------------------------------- #


def build_lighthouse_with_rotating_beacon_assembly(
    config: LighthouseWithRotatingBeaconAssemblyConfig,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name="lighthouse_with_rotating_beacon_assembly", assets=assets)
    for material, rgba in r.palette.items():
        model.material(material, rgba=rgba)

    # Tower body (root).
    tower = model.part("tower")
    _build_tower_body(tower, r)
    if r.lantern_housing_style == "baked_into_body":
        _bake_lantern_visuals(tower, r)
    else:
        _bake_minimal_lantern_anchor(tower, r)
    _bake_door_anchor_visuals(tower, r)
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=r.tower_base_radius, length=r.tower_height),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, r.tower_height * 0.5)),
    )

    # Slot B: lantern stack parts.
    if r.lantern_housing_style == "separate_lantern_stack":
        _emit_lantern_room_part(model, r, name="lantern_room")
        _emit_roof_cap_part(model, r)
        # tower -> lantern_room FIXED (joint origin at top of tower's gallery deck)
        model.articulation(
            "tower_to_lantern_room",
            ArticulationType.FIXED,
            parent=tower,
            child=model.get_part("lantern_room"),
            origin=Origin(xyz=(0.0, 0.0, r.lantern_z)),
        )
        # lantern_room -> roof_cap FIXED (joint origin at top of lantern_room)
        model.articulation(
            "lantern_room_to_roof_cap",
            ArticulationType.FIXED,
            parent=model.get_part("lantern_room"),
            child=model.get_part("roof_cap"),
            origin=Origin(xyz=(0.0, 0.0, r.lantern_height)),
        )
    elif r.lantern_housing_style == "full_fixed_stack":
        _emit_gallery_part(model, r)
        _emit_lantern_room_part(model, r, name="lantern_room")
        _emit_roof_cap_part(model, r)
        model.articulation(
            "tower_to_gallery",
            ArticulationType.FIXED,
            parent=tower,
            child=model.get_part("gallery"),
            origin=Origin(xyz=(0.0, 0.0, r.gallery_z)),
        )
        model.articulation(
            "gallery_to_lantern_room",
            ArticulationType.FIXED,
            parent=model.get_part("gallery"),
            child=model.get_part("lantern_room"),
            origin=Origin(xyz=(0.0, 0.0, 0.10)),
        )
        model.articulation(
            "lantern_room_to_roof_cap",
            ArticulationType.FIXED,
            parent=model.get_part("lantern_room"),
            child=model.get_part("roof_cap"),
            origin=Origin(xyz=(0.0, 0.0, r.lantern_height)),
        )

    # Slot D: beacon bearing topology.
    bearing = r.beacon_bearing_topology
    beacon_parent_name = "tower"
    beacon_origin_z = r.beacon_base_z
    if bearing == "separate_fixed_pedestal_part":
        _emit_pedestal_part(model, r)
        # Decide which "tower side" part to FIXED-attach the pedestal to.
        if r.lantern_housing_style in ("separate_lantern_stack", "full_fixed_stack"):
            attach_to_part_name = "lantern_room"
            attach_origin_z = 0.10  # just above lantern_room sill
        else:
            attach_to_part_name = "tower"
            attach_origin_z = r.beacon_base_z
        model.articulation(
            f"{attach_to_part_name}_to_pedestal",
            ArticulationType.FIXED,
            parent=model.get_part(attach_to_part_name),
            child=model.get_part("pedestal"),
            origin=Origin(xyz=(0.0, 0.0, attach_origin_z)),
        )
        beacon_parent_name = "pedestal"
        # Beacon sits on the pedestal bearing cap (top of pedestal)
        ped_h = max(0.30, r.lantern_height * 0.55)
        beacon_origin_z = ped_h
    elif bearing == "separate_fixed_shaft_part":
        _emit_central_shaft_part(model, r)
        if r.lantern_housing_style in ("separate_lantern_stack", "full_fixed_stack"):
            attach_to_part_name = "lantern_room"
            attach_origin_z = 0.05
        else:
            # Non-stack lantern is always `baked_into_body` here, so the tower
            # carries the baked gallery deck. Rest the shaft foot on the deck top
            # (real contact surface) rather than a fixed offset below the optic —
            # a fixed offset drifted into the gap above the deck at some scales.
            attach_to_part_name = "tower"
            attach_origin_z = r.gallery_z + _BAKED_GALLERY_DECK_H
        model.articulation(
            f"{attach_to_part_name}_to_central_shaft",
            ArticulationType.FIXED,
            parent=model.get_part(attach_to_part_name),
            child=model.get_part("central_shaft"),
            origin=Origin(xyz=(0.0, 0.0, attach_origin_z)),
        )
        beacon_parent_name = "central_shaft"
        # Beacon sleeve wraps the central shaft; sit beacon at z=0.05 on shaft.
        beacon_origin_z = 0.05

    # Beacon part (CONTINUOUS).
    sign = r.beacon_axis_sign
    # Decide beacon parent and its joint-origin z in that parent's local frame.
    if bearing == "direct_to_body" and r.lantern_housing_style in (
        "separate_lantern_stack",
        "full_fixed_stack",
    ):
        # Parent the beacon to lantern_room so the joint origin sits on its
        # baked spindle; the lantern_room part frame has its sill at z=0.
        beacon_parent_name = "lantern_room"
        beacon_origin_z = min(r.lantern_height * 0.30, 0.30)
        # Headroom = lantern_height - beacon_origin_z (above-origin space).
        headroom = max(0.30, r.lantern_height - beacon_origin_z - 0.05)
    elif bearing == "separate_fixed_pedestal_part":
        # beacon_origin_z is the top of the pedestal in pedestal-local frame.
        ped_h = max(0.30, r.lantern_height * 0.55)
        beacon_origin_z = ped_h
        # Headroom = vertical room above pedestal top inside lantern envelope.
        # Beacon parent is pedestal — but the world headroom check still applies.
        headroom = max(0.30, r.lantern_height - 0.05)
    elif bearing == "separate_fixed_shaft_part":
        # Keep the optic at beacon_base_z regardless of where the shaft foot
        # welds: for the tower attach it now rests on the deck top, so lift the
        # beacon on the shaft by the same amount (shaft is tall enough).
        if attach_to_part_name == "tower":
            beacon_origin_z = max(0.05, r.beacon_base_z - attach_origin_z)
        else:
            beacon_origin_z = 0.05
        headroom = max(0.30, r.lantern_height - 0.05)
    else:  # direct_to_body + baked_into_body
        beacon_origin_z = r.beacon_base_z
        # World headroom: lantern_top_z - beacon_origin_z
        headroom = max(0.30, r.lantern_top_z - r.beacon_base_z - 0.05)

    _build_beacon_part(model, r, headroom=headroom)
    parent_part = model.get_part(beacon_parent_name)
    model.articulation(
        "beacon_spin",
        ArticulationType.CONTINUOUS,
        parent=parent_part,
        child=model.get_part("beacon"),
        origin=Origin(xyz=(0.0, 0.0, beacon_origin_z)),
        axis=(0.0, 0.0, float(sign)),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    # Slot E: access opening (REVOLUTE).
    if r.access_opening != "none":
        name, jorigin, jaxis, parent_name = _emit_door_part(model, r, kind=r.access_opening)
        model.articulation(
            f"{parent_name}_to_{name}",
            ArticulationType.REVOLUTE,
            parent=model.get_part(parent_name),
            child=model.get_part(name),
            origin=Origin(xyz=jorigin),
            axis=jaxis,
            motion_limits=MotionLimits(
                effort=1.0, velocity=2.0, lower=0.0, upper=r.door_open_upper
            ),
        )

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_lighthouse_with_rotating_beacon_assembly(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_lighthouse_with_rotating_beacon_assembly(config_from_seed(seed), assets=assets)


def slot_choices_for_config(
    r: ResolvedLighthouseWithRotatingBeaconAssemblyConfig,
) -> list[tuple[str, str]]:
    return [
        ("tower_body", r.tower_body_style),
        ("lantern_housing", r.lantern_housing_style),
        ("beacon_optic", r.beacon_optic_style),
        ("beacon_bearing", r.beacon_bearing_topology),
        ("access_opening", r.access_opening),
    ]


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


# --------------------------------------------------------------------------- #
# Author-layer QC
# --------------------------------------------------------------------------- #


def _declare_captured_overlaps(ctx: TestContext, model: ArticulatedObject) -> None:
    names = [p.name for p in model.parts]
    name_set = set(names)
    # Beacon overlaps baked spindle on tower / lantern_room / shaft / pedestal cap.
    if "beacon" in name_set:
        for parent in ("tower", "lantern_room", "pedestal", "central_shaft"):
            if parent in name_set:
                ctx.allow_overlap(
                    model.get_part(parent),
                    model.get_part("beacon"),
                    reason="beacon turntable / lamp_column wraps the bearing spindle (captured rotation)",
                )
    # Door hinge barrel sits on its host (tower/lantern_room) surface — declare
    # an inter-part overlap allowance for all access-opening children.
    for door_name in ("lantern_door", "gallery_gate", "hatch", "trap_door"):
        if door_name in name_set:
            for host in ("tower", "lantern_room", "gallery", "roof_cap"):
                if host in name_set:
                    ctx.allow_overlap(
                        model.get_part(host),
                        model.get_part(door_name),
                        reason=f"{door_name} hinge barrel intersects {host} along the hinge line",
                    )
    # Stacked / inserted bearing parts intersect host visuals where they
    # rise inside the lantern envelope.
    for child in ("lantern_room", "roof_cap", "gallery", "pedestal", "central_shaft"):
        if child in name_set:
            for host in ("tower", "gallery", "lantern_room", "roof_cap"):
                if host in name_set and host != child:
                    ctx.allow_overlap(
                        model.get_part(host),
                        model.get_part(child),
                        reason=f"{child} sits inside / on {host} (FIXED stacked bearing or lantern part)",
                    )
    # Beacon also intersects ancillary lantern visuals (e.g. roof_cap when the
    # beacon top approaches the ceiling, or gallery_rail when beacon centered
    # over deck) — declare permissively.
    if "beacon" in name_set:
        for other in ("gallery", "roof_cap", "lantern_door", "hatch", "gallery_gate", "trap_door"):
            if other in name_set:
                ctx.allow_overlap(
                    model.get_part(other),
                    model.get_part("beacon"),
                    reason=f"beacon optic may sweep through {other} envelope as it spins",
                )


def run_lighthouse_with_rotating_beacon_assembly_tests(
    object_model: ArticulatedObject,
    config: LighthouseWithRotatingBeaconAssemblyConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    _declare_captured_overlaps(ctx, object_model)

    ctx.check_model_valid()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.030)
    ctx.fail_if_joint_mating_has_gap()

    # Identity checks.
    parts = {p.name for p in object_model.parts}
    ctx.check("tower_present", "tower" in parts)
    ctx.check("beacon_present", "beacon" in parts)
    spin_joints = [
        j
        for j in object_model.articulations
        if j.articulation_type == ArticulationType.CONTINUOUS and j.child == "beacon"
    ]
    ctx.check(
        "exactly_one_continuous_beacon_spin",
        len(spin_joints) == 1,
        details=f"expected 1 CONTINUOUS beacon joint, got {len(spin_joints)}",
    )
    if spin_joints:
        ax = spin_joints[0].axis
        ctx.check(
            "beacon_spin_axis_vertical",
            ax[0] == 0.0 and ax[1] == 0.0 and ax[2] in (1.0, -1.0),
            details=f"beacon spin axis must be (0,0,±1); got {ax!r}",
        )

    # If access_opening != none, expect exactly one REVOLUTE joint.
    revolutes = [
        j for j in object_model.articulations if j.articulation_type == ArticulationType.REVOLUTE
    ]
    if r.access_opening == "none":
        ctx.check(
            "no_revolute_when_no_access_opening",
            len(revolutes) == 0,
            details=f"expected 0 REVOLUTE joints (access_opening=none), got {len(revolutes)}",
        )
    else:
        ctx.check(
            "exactly_one_revolute_for_access_opening",
            len(revolutes) == 1,
            details=f"expected 1 REVOLUTE joint, got {len(revolutes)}",
        )

    return ctx.report()


__all__ = [
    "AccessOpening",
    "BeaconBearingTopology",
    "BeaconOpticStyle",
    "LanternHousingStyle",
    "LighthouseWithRotatingBeaconAssemblyConfig",
    "PaletteTheme",
    "ResolvedLighthouseWithRotatingBeaconAssemblyConfig",
    "TowerBodyStyle",
    "__modular__",
    "build_lighthouse_with_rotating_beacon_assembly",
    "build_seeded_lighthouse_with_rotating_beacon_assembly",
    "config_from_seed",
    "resolve_config",
    "run_lighthouse_with_rotating_beacon_assembly_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
]
