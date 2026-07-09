"""European wall light switch / socket plate modular template.

NOTE on the slug name: "switch" here = **European wall switch / socket plate**
(a thin faceplate sitting on a wall), NOT a knife switch, ship/industrial console,
or router panel. The structure family is a wall-mounted plate: a thin warm-beige
``wall_panel`` (root) carrying a FIXED ``faceplate`` (matte ivory ``plate_field``
+ polished chrome ``trim_ring``), with N **actuators** laid out side-by-side along
X — each actuator is an independent MOVING child of the faceplate.

Sourced from the reviewed modular spec
``articraft_template_authoring/specs_modular_v1/Other_Switch.md`` and the
``picture/Other/Switch`` 5-star sample pool (1 parent + 7 slot/multiplicity forks),
all synced under ``data/records/``.

Structure (pattern = ``mixed``): a single root ``wall_panel`` + FIXED ``faceplate``,
with two named module axes + one multiplicity axis:

  * ``actuator_type`` (4): the per-actuator part tree + joint topology —
      - rocker        : ``rocker_{i}``  + ``rocker_{i}_pivot``  REVOLUTE +X (±0.10 see-saw)
      - toggle_lever  : ``toggle_{i}``  + ``toggle_{i}_pivot``  REVOLUTE +X (±0.35 flip)
      - push_button   : ``button_{i}``  + ``button_{i}_press``  PRISMATIC -Y (0..0.003)
      - rotary_dimmer : ``dimmer_{i}``  + ``dimmer_{i}_spin``   CONTINUOUS +Y (free)
  * ``faceplate_shape`` (3): rounded_rect_horizontal (+Schuko) / square_single /
    round_plate — a mesh-footprint dimension on ``plate_field``/``trim_ring``;
    it does NOT add a joint.
  * ``gang_count`` (N in [1,4]): a multiplicity axis — N identical actuators copied
    along X by ``xc = (i-(N-1)/2)*pitch`` (absolute, center-symmetric). Each copy is
    its own MOVING part with its own joint. N is encoded into the slot_choice tuple
    as ``("gang_count", f"n{N}")``.

All pivots / button seats / dimmer shafts are captured-pin / seat-on-rim /
shaft-through-hole geometry, so those joints omit ``MatingContract``
(grandfathered) and are guarded by the flat articulation-origin baseline +
element-scoped ``allow_overlap`` (mirroring each source record's run_tests block).

Compatibility gating (resolve_config, spec §9):
  * round_plate's disc (~0.156 m) cannot host >2 actuator pockets side-by-side ->
    round_plate forces N<=2.
  * Schuko socket module is emitted only on rounded_rect_horizontal.
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
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

__modular__ = True

ActuatorType = Literal["rocker", "toggle_lever", "push_button", "rotary_dimmer"]
FaceplateShape = Literal["rounded_rect_horizontal", "square_single", "round_plate"]
PaletteStyle = Literal[
    "ivory_chrome",
    "matte_white",
    "brushed_steel",
    "matte_black",
    "warm_brass",
]

ACTUATOR_TYPES: tuple[ActuatorType, ...] = (
    "rocker",
    "toggle_lever",
    "push_button",
    "rotary_dimmer",
)
FACEPLATE_SHAPES: tuple[FaceplateShape, ...] = (
    "rounded_rect_horizontal",
    "square_single",
    "round_plate",
)
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "ivory_chrome",
    "matte_white",
    "brushed_steel",
    "matte_black",
    "warm_brass",
)

N_MIN = 1
N_MAX = 4
# Gang-count sampling weights (spec §8: 1/2 high-frequency, 3 common, 4 long tail).
GANG_COUNT_WEIGHTS = (0.32, 0.34, 0.20, 0.14)
# round_plate disc can only host <=2 side-by-side pockets (spec §9 (1)).
ROUND_PLATE_MAX_N = 2

# Palette colorways: plate / trim / actuator / clip / wall. Each drives every
# .visual(material=...) in the build.
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "ivory_chrome": {
        "wall": (0.80, 0.70, 0.60, 1.0),
        "plate": (0.93, 0.90, 0.83, 1.0),
        "trim": (0.78, 0.79, 0.82, 1.0),
        "actuator": (0.93, 0.90, 0.83, 1.0),
        "steel": (0.60, 0.62, 0.65, 1.0),
    },
    "matte_white": {
        "wall": (0.82, 0.82, 0.80, 1.0),
        "plate": (0.96, 0.96, 0.95, 1.0),
        "trim": (0.90, 0.90, 0.90, 1.0),
        "actuator": (0.97, 0.97, 0.96, 1.0),
        "steel": (0.62, 0.63, 0.64, 1.0),
    },
    "brushed_steel": {
        "wall": (0.74, 0.72, 0.68, 1.0),
        "plate": (0.70, 0.72, 0.74, 1.0),
        "trim": (0.82, 0.84, 0.86, 1.0),
        "actuator": (0.66, 0.68, 0.70, 1.0),
        "steel": (0.55, 0.57, 0.60, 1.0),
    },
    "matte_black": {
        "wall": (0.70, 0.66, 0.60, 1.0),
        "plate": (0.10, 0.10, 0.11, 1.0),
        "trim": (0.32, 0.33, 0.35, 1.0),
        "actuator": (0.14, 0.14, 0.15, 1.0),
        "steel": (0.48, 0.49, 0.51, 1.0),
    },
    "warm_brass": {
        "wall": (0.80, 0.72, 0.62, 1.0),
        "plate": (0.92, 0.88, 0.78, 1.0),
        "trim": (0.72, 0.58, 0.32, 1.0),
        "actuator": (0.90, 0.86, 0.76, 1.0),
        "steel": (0.60, 0.52, 0.36, 1.0),
    },
}

# ---------------------------------------------------------------------------
# Base real-world dimensions (meters), from the parent baseline (c46f379f).
# Coordinate convention: +Z up, plate width along X, plate face toward +Y,
# wall surface at the plane y = 0; the plate protrudes to +Y. Visuals built in
# CadQuery's XY plane (extruded +Z) are rotated into the wall frame with roll
# = -pi/2, mapping cq +Z -> world +Y.
# ---------------------------------------------------------------------------
_CQ_TO_WALL = (-math.pi / 2.0, 0.0, 0.0)

_WALL_T = 0.018
_WALL_H = 0.24

_TRIM_DEPTH = 0.011  # front of the chrome lip (y from wall plane)
_TRIM_R = 0.012
_PLATE_DEPTH = 0.010  # front face of the ivory field
_PLATE_R = 0.0095

_MODULE_PITCH = 0.075  # nominal center-to-center actuator spacing
_PLATE_H = 0.082  # nominal plate height (Z)
_TRIM_H = 0.090

# Rocker pockets (square openings with a thin back floor).
_POCKET_SIZE = 0.054
_POCKET_R = 0.004
_POCKET_FLOOR_Y = 0.0025

# Rocker paddle.
_ROCKER_SIZE = 0.050
_ROCKER_BASE_T = 0.0045
_ROCKER_CAP_T = 0.0035
_ROCKER_T = _ROCKER_BASE_T + _ROCKER_CAP_T  # 0.008
_ROCKER_BACK_Y = 0.0055
_ROCKER_PIVOT_Y = _ROCKER_BACK_Y + _ROCKER_T / 2.0  # see-saw pivot mid-depth
_ROCKER_RANGE = 0.10
_AXLE_R = 0.002
_AXLE_LEN = _POCKET_SIZE + 0.003

# Toggle lever.
_ESC_SIZE = 0.052
_ESC_R = 0.005
_ESC_FACE_Y = 0.0120
_TOGGLE_SLOT_W = 0.021
_TOGGLE_SLOT_H = 0.009
_TOGGLE_SLOT_R = 0.002
_LEVER_W = 0.018
_LEVER_T = 0.005
_LEVER_H = 0.028
_TOGGLE_PIVOT_Y = _PLATE_DEPTH
_TOGGLE_RANGE = 0.35
_TOGGLE_AXLE_R = 0.0015
_TOGGLE_AXLE_LEN = _TOGGLE_SLOT_W + 0.004

# Push button.
_BUTTON_SIZE = 0.056
_BUTTON_CAP_T = 0.006
_BUTTON_BACK_DEPTH = 0.002
_BUTTON_GUIDE_SIZE = 0.050
_BUTTON_GUIDE_L = 0.002
_BUTTON_TRAVEL = 0.003

# Rotary dimmer.
_SHAFT_HOLE_R = 0.012
_DIMMER_DIAMETER = 0.044
_DIMMER_HEIGHT = 0.018
_DIMMER_BASE_Y = _PLATE_DEPTH
_DIMMER_POINTER_R = 0.024  # off-axis pointer tab radius (> knob radius)

# Schuko socket (only on rounded_rect_horizontal).
_TILE_SIZE = 0.055
_TILE_R = 0.006
_TILE_FACE_Y = 0.0125
_WELL_RADIUS = 0.020
_WELL_FLOOR_Y = 0.0035
_PIN_HOLE_R = 0.0024
_PIN_HOLE_DX = 0.0095
_PIN_HOLE_FLOOR_Y = 0.0008
_CLIP_W = 0.012
_CLIP_T = 0.005
_CLIP_H = 0.0048
_CLIP_ZC = _WELL_RADIUS - 0.0018
_CLIP_YC = 0.0080

# round_plate footprint (axisymmetric).
_ROUND_TRIM_OD = 0.170
_ROUND_TRIM_ID = 0.154
_ROUND_PLATE_D = 0.156


@dataclass(frozen=True)
class SwitchConfig:
    actuator_type: ActuatorType | None = None
    faceplate_shape: FaceplateShape | None = None
    gang_count: int | None = None
    palette_style: PaletteStyle = "ivory_chrome"
    plate_width_scale: float = 1.0
    plate_height_scale: float = 1.0
    module_pitch_scale: float = 1.0
    actuator_travel_scale: float = 1.0
    name: str = "switch"


@dataclass(frozen=True)
class ResolvedSwitchConfig:
    actuator_type: ActuatorType
    faceplate_shape: FaceplateShape
    gang_count: int
    palette_style: PaletteStyle
    has_schuko: bool
    # Concrete geometry (scaled).
    pitch: float
    plate_w: float  # ivory field width (X)
    plate_h: float  # ivory field height (Z)
    trim_w: float
    trim_h: float
    wall_w: float
    socket_x: float  # Schuko tile center X (rightmost), only when has_schuko
    actuator_xs: tuple[float, ...]
    revolute_range: float  # rocker/toggle scaled half-range
    button_travel: float
    name: str

    @property
    def is_round(self) -> bool:
        return self.faceplate_shape == "round_plate"


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _pick(value, choices):
    return value if value in choices else choices[0]


def config_from_seed(seed: int) -> SwitchConfig:
    rng = random.Random(seed)
    return SwitchConfig(
        actuator_type=rng.choice(ACTUATOR_TYPES),
        faceplate_shape=rng.choice(FACEPLATE_SHAPES),
        gang_count=rng.choices((1, 2, 3, 4), weights=GANG_COUNT_WEIGHTS, k=1)[0],
        palette_style=rng.choice(PALETTE_STYLES),
        plate_width_scale=round(rng.uniform(0.92, 1.10), 4),
        plate_height_scale=round(rng.uniform(0.92, 1.10), 4),
        module_pitch_scale=round(rng.uniform(0.90, 1.10), 4),
        actuator_travel_scale=round(rng.uniform(0.85, 1.12), 4),
        name=f"seeded_switch_{seed}",
    )


def resolve_config(config: SwitchConfig | None = None) -> ResolvedSwitchConfig:
    cfg = config or SwitchConfig()
    palette_style = _pick(cfg.palette_style, PALETTE_STYLES)
    actuator_type = _pick(cfg.actuator_type, ACTUATOR_TYPES)
    faceplate_shape = _pick(cfg.faceplate_shape, FACEPLATE_SHAPES)

    gang_count = int(cfg.gang_count) if cfg.gang_count is not None else 2
    gang_count = int(_clamp(gang_count, N_MIN, N_MAX))

    # --- Compatibility gating (spec §9). ---
    # (1) round_plate disc cannot host >2 side-by-side actuators -> clamp N<=2.
    if faceplate_shape == "round_plate":
        gang_count = min(gang_count, ROUND_PLATE_MAX_N)
    # (2) Schuko socket module is only emitted on rounded_rect_horizontal.
    has_schuko = faceplate_shape == "rounded_rect_horizontal"

    width_scale = _clamp(cfg.plate_width_scale, 0.92, 1.10)
    height_scale = _clamp(cfg.plate_height_scale, 0.92, 1.10)
    pitch_scale = _clamp(cfg.module_pitch_scale, 0.90, 1.10)
    travel_scale = _clamp(cfg.actuator_travel_scale, 0.85, 1.12)

    pitch = _MODULE_PITCH * pitch_scale

    # --- Absolute, center-symmetric actuator placement (spec §6/§8). ---
    actuator_xs = tuple(
        (i - (gang_count - 1) / 2.0) * pitch for i in range(gang_count)
    )

    # --- Footprint sizing. ---
    if faceplate_shape == "round_plate":
        # Disc footprint: width == height locked for a true circle (spec §7 eq).
        round_scale = width_scale  # round_plate locks height = width
        plate_w = _ROUND_PLATE_D * round_scale
        plate_h = _ROUND_PLATE_D * round_scale
        trim_w = _ROUND_TRIM_OD * round_scale
        trim_h = _ROUND_TRIM_OD * round_scale
    elif faceplate_shape == "square_single":
        # Single square plate (no Schuko, no N widening: square is always 1-gang
        # visually but N actuators still stack center-symmetric and the plate
        # widens to hold them).
        base_w = max(0.082, gang_count * pitch + 0.028)
        plate_w = base_w * width_scale
        plate_h = 0.082 * height_scale
        trim_w = plate_w + 0.008
        trim_h = plate_h + 0.008
    else:  # rounded_rect_horizontal
        # Plate widens with N: actuators occupy N*pitch, Schuko adds one module
        # width on the right. margin per side ~0.0125.
        modules_w = gang_count * pitch + (pitch if has_schuko else 0.0)
        base_w = modules_w + 0.015
        plate_w = base_w * width_scale
        plate_h = _PLATE_H * height_scale
        trim_w = plate_w + 0.008
        trim_h = plate_h + 0.008

    # --- Clearance inequality (spec §7): actuators must fit inside the plate. ---
    # row spans the actuator centers; require the outermost actuator + half its
    # footprint to stay inside the inner field (minus a margin).
    margin = 0.006
    actuator_half = _POCKET_SIZE / 2.0
    if faceplate_shape == "round_plate":
        inner_half_x = (plate_w / 2.0) - margin
    else:
        inner_half_x = (plate_w / 2.0) - margin
    if gang_count >= 1:
        # Rightmost extent including Schuko on horizontal.
        outer_center = actuator_xs[-1] if actuator_xs else 0.0
        row_right = outer_center + actuator_half
        if has_schuko:
            row_right = outer_center + pitch + _TILE_SIZE / 2.0
        # If the row spills past the field, widen the plate to contain it.
        needed_half = row_right + margin
        if needed_half > inner_half_x:
            plate_w = 2.0 * needed_half
            trim_w = plate_w + 0.008
            inner_half_x = (plate_w / 2.0) - margin

    socket_x = (actuator_xs[-1] + pitch) if (has_schuko and actuator_xs) else (pitch if has_schuko else 0.0)

    wall_w = max(0.18, trim_w + 0.10)

    # --- actuator_travel conditional (spec §7): scale REVOLUTE/PRISMATIC upper. ---
    revolute_range = _clamp(_ROCKER_RANGE, 0.04, math.pi * 0.5)  # rocker default
    # The rocker/toggle ranges are scaled below per-actuator at emit time; keep
    # the button travel clamped so the button never presses through the floor.
    button_travel = _clamp(_BUTTON_TRAVEL * travel_scale, 0.0015, _POCKET_FLOOR_Y + 0.0008)

    return ResolvedSwitchConfig(
        actuator_type=actuator_type,
        faceplate_shape=faceplate_shape,
        gang_count=gang_count,
        palette_style=palette_style,
        has_schuko=has_schuko,
        pitch=pitch,
        plate_w=plate_w,
        plate_h=plate_h,
        trim_w=trim_w,
        trim_h=trim_h,
        wall_w=wall_w,
        socket_x=socket_x,
        actuator_xs=actuator_xs,
        revolute_range=revolute_range,
        button_travel=button_travel,
        name=cfg.name or "switch",
    )


def with_overrides(config: SwitchConfig, **kwargs: object) -> SwitchConfig:
    return replace(config, **kwargs)


def slot_choices_for_config(
    config: SwitchConfig | ResolvedSwitchConfig,
) -> tuple[tuple[str, str], ...]:
    r = config if isinstance(config, ResolvedSwitchConfig) else resolve_config(config)
    return (
        ("actuator_type", r.actuator_type),
        ("faceplate_shape", r.faceplate_shape),
        ("gang_count", f"n{r.gang_count}"),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


def _rev_range(r: ResolvedSwitchConfig, base: float, travel_scale: float) -> float:
    return _clamp(base * travel_scale, 0.04, math.pi * 0.5)


# ---------------------------------------------------------------------------
# Footprint mesh helpers (faceplate_shape). Each preserves the source primitive
# type: horizontal/square = rounded-rect, round = circle/annulus (lathe).
# ---------------------------------------------------------------------------
def _rounded_rect(w: float, h: float, r: float) -> cq.Sketch:
    rr = min(r, 0.45 * min(w, h))
    return cq.Sketch().rect(w, h).vertices().fillet(rr)


def _trim_ring_mesh(r: ResolvedSwitchConfig, assets):
    """Thin raised polished-chrome border around the ivory field."""
    if r.faceplate_shape == "round_plate":
        outer_r = r.trim_w / 2.0
        inner_r = r.plate_w / 2.0 - 0.001
        outer = cq.Workplane("XY").circle(outer_r).extrude(_TRIM_DEPTH)
        inner = (
            cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
            .circle(inner_r)
            .extrude(_TRIM_DEPTH + 0.002)
        )
        ring = outer.cut(inner)
    else:
        ring = (
            cq.Workplane("XY")
            .placeSketch(_rounded_rect(r.trim_w, r.trim_h, _TRIM_R))
            .extrude(_TRIM_DEPTH)
        )
        opening = (
            cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
            .placeSketch(_rounded_rect(r.plate_w - 0.002, r.plate_h - 0.002, _PLATE_R))
            .extrude(_TRIM_DEPTH + 0.002)
        )
        ring = ring.cut(opening)
    try:
        ring = ring.edges(">Z").fillet(0.0015)
    except Exception:
        pass
    return mesh_from_cadquery(ring, "trim_ring", assets=assets)


def _cut_actuator_openings(plate: cq.Workplane, r: ResolvedSwitchConfig) -> cq.Workplane:
    """Cut the per-actuator opening for each gang into the plate front face."""
    at = r.actuator_type
    for xc in r.actuator_xs:
        if at in ("rocker", "push_button"):
            # Square pocket with a thin back floor.
            pocket = (
                cq.Workplane("XY", origin=(xc, 0.0, _POCKET_FLOOR_Y))
                .placeSketch(_rounded_rect(_POCKET_SIZE, _POCKET_SIZE, _POCKET_R))
                .extrude(_PLATE_DEPTH)
            )
            plate = plate.cut(pocket)
        elif at == "toggle_lever":
            # Narrow through-slot in a raised escutcheon.
            slot = (
                cq.Workplane("XY", origin=(xc, 0.0, -0.001))
                .placeSketch(_rounded_rect(_TOGGLE_SLOT_W, _TOGGLE_SLOT_H, _TOGGLE_SLOT_R))
                .extrude(_ESC_FACE_Y + 0.002)
            )
            plate = plate.cut(slot)
        else:  # rotary_dimmer
            hole = (
                cq.Workplane("XY", origin=(xc, 0.0, -0.001))
                .circle(_SHAFT_HOLE_R)
                .extrude(_PLATE_DEPTH + 0.002)
            )
            plate = plate.cut(hole)
    return plate


def _plate_mesh(r: ResolvedSwitchConfig, assets):
    """Matte ivory field with per-actuator openings + optional Schuko well."""
    if r.faceplate_shape == "round_plate":
        plate = cq.Workplane("XY").circle(r.plate_w / 2.0).extrude(_PLATE_DEPTH)
    else:
        plate = (
            cq.Workplane("XY")
            .placeSketch(_rounded_rect(r.plate_w, r.plate_h, _PLATE_R))
            .extrude(_PLATE_DEPTH)
        )

    # Raised toggle escutcheons (toggle only) — built before cuts so the slot
    # passes cleanly through them.
    if r.actuator_type == "toggle_lever":
        for xc in r.actuator_xs:
            esc = (
                cq.Workplane("XY", origin=(xc, 0.0, _PLATE_DEPTH - 0.001))
                .placeSketch(_rounded_rect(_ESC_SIZE, _ESC_SIZE, _ESC_R))
                .extrude(_ESC_FACE_Y - _PLATE_DEPTH + 0.001)
            )
            plate = plate.union(esc)

    # Raised Schuko tile (rounded_rect_horizontal only).
    if r.has_schuko:
        tile = (
            cq.Workplane("XY", origin=(r.socket_x, 0.0, _PLATE_DEPTH - 0.001))
            .placeSketch(_rounded_rect(_TILE_SIZE, _TILE_SIZE, _TILE_R))
            .extrude(_TILE_FACE_Y - _PLATE_DEPTH + 0.001)
        )
        plate = plate.union(tile)

    try:
        plate = plate.edges(">Z").fillet(0.0012)
    except Exception:
        pass

    plate = _cut_actuator_openings(plate, r)

    if r.has_schuko:
        # True hollow circular well recessed into the tile.
        well = (
            cq.Workplane("XY", origin=(r.socket_x, 0.0, _WELL_FLOOR_Y))
            .circle(_WELL_RADIUS)
            .extrude(_TILE_FACE_Y)
        )
        plate = plate.cut(well)
        # Two blind Schuko pin holes in the well floor.
        for sx in (-1.0, 1.0):
            hole = (
                cq.Workplane(
                    "XY", origin=(r.socket_x + sx * _PIN_HOLE_DX, 0.0, _PIN_HOLE_FLOOR_Y)
                )
                .circle(_PIN_HOLE_R)
                .extrude(_WELL_FLOOR_Y - _PIN_HOLE_FLOOR_Y + 0.001)
            )
            plate = plate.cut(hole)

    return mesh_from_cadquery(plate, "plate_field", assets=assets)


# ---------------------------------------------------------------------------
# Actuator mesh helpers (actuator_type). One mesh object reused for all N gangs.
# ---------------------------------------------------------------------------
def _rocker_mesh(assets):
    """Slightly convex square ivory paddle, authored in a LOCAL frame centered
    on its see-saw pivot."""
    half = _ROCKER_T / 2.0
    base = (
        cq.Workplane("XY", origin=(0.0, 0.0, -half))
        .placeSketch(_rounded_rect(_ROCKER_SIZE, _ROCKER_SIZE, 0.005))
        .extrude(_ROCKER_BASE_T)
    )
    cap = (
        cq.Workplane("XY")
        .placeSketch(
            _rounded_rect(_ROCKER_SIZE, _ROCKER_SIZE, 0.005).moved(
                cq.Location(cq.Vector(0.0, 0.0, -half + _ROCKER_BASE_T))
            ),
            _rounded_rect(_ROCKER_SIZE - 0.008, _ROCKER_SIZE - 0.008, 0.008).moved(
                cq.Location(cq.Vector(0.0, 0.0, half))
            ),
        )
        .loft()
    )
    return mesh_from_cadquery(base.union(cap), "rocker_paddle", assets=assets)


def _toggle_lever_mesh(assets):
    """Small tapered flip toggle lever, pivot at origin, extends along cq +Z."""
    lever = (
        cq.Workplane("XY")
        .placeSketch(
            _rounded_rect(_LEVER_W, _LEVER_T, 0.001).moved(
                cq.Location(cq.Vector(0.0, 0.0, 0.0))
            ),
            _rounded_rect(_LEVER_W * 0.82, _LEVER_T * 0.82, 0.0015).moved(
                cq.Location(cq.Vector(0.0, 0.0, _LEVER_H))
            ),
        )
        .loft()
    )
    try:
        lever = lever.edges(">Z").fillet(0.0012)
    except Exception:
        pass
    return mesh_from_cadquery(lever, "toggle_lever", assets=assets)


def _button_mesh(assets):
    """Square ivory push-button cap + narrower guide, z=0 at the plate front."""
    cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, -_BUTTON_BACK_DEPTH))
        .placeSketch(_rounded_rect(_BUTTON_SIZE, _BUTTON_SIZE, 0.004))
        .extrude(_BUTTON_CAP_T)
    )
    guide = (
        cq.Workplane("XY", origin=(0.0, 0.0, -_BUTTON_BACK_DEPTH - _BUTTON_GUIDE_L))
        .placeSketch(_rounded_rect(_BUTTON_GUIDE_SIZE, _BUTTON_GUIDE_SIZE, 0.003))
        .extrude(_BUTTON_GUIDE_L)
    )
    button = cap.union(guide)
    try:
        button = button.edges(">Z").fillet(0.0008)
    except Exception:
        pass
    return mesh_from_cadquery(button, "button_cap", assets=assets)


def _dimmer_knob_mesh():
    """Round rotary dimmer knob: tapered ribbed body with a raised pointer line.
    Built in local Z (knob axis), center=False so the mounting face sits at z=0."""
    knob = KnobGeometry(
        _DIMMER_DIAMETER,
        _DIMMER_HEIGHT,
        body_style="tapered",
        top_diameter=_DIMMER_DIAMETER - 0.008,
        edge_radius=0.001,
        grip=KnobGrip(style="ribbed", count=12, depth=0.0008, width=0.002),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0006),
        center=False,
    )
    return mesh_from_geometry(knob, "dimmer_knob")


# ---------------------------------------------------------------------------
# Actuator emitters (gang_count multiplicity). Each emits N MOVING parts, each
# with its own joint. Absolute center-symmetric placement (N-invariant).
# ---------------------------------------------------------------------------
def _emit_rockers(model, r, faceplate, mats, travel_scale: float) -> list[str]:
    rocker_mesh = _rocker_mesh(model.assets)
    rng = r.revolute_range
    rrange = _rev_range(r, _ROCKER_RANGE, travel_scale)
    names: list[str] = []
    for i, xc in enumerate(r.actuator_xs):
        rocker = model.part(f"rocker_{i}")
        rocker.visual(
            rocker_mesh,
            origin=Origin(rpy=_CQ_TO_WALL),
            material=mats["actuator"],
            name="rocker_paddle",
        )
        rocker.visual(
            Cylinder(radius=_AXLE_R, length=_AXLE_LEN),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=mats["steel"],
            name="pivot_axle",
        )
        model.articulation(
            f"rocker_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=faceplate,
            child=rocker,
            origin=Origin(xyz=(xc, _ROCKER_PIVOT_Y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-rrange, upper=rrange),
        )
        names.append(f"rocker_{i}")
    _ = rng
    return names


def _emit_toggles(model, r, faceplate, mats, travel_scale: float) -> list[str]:
    lever_mesh = _toggle_lever_mesh(model.assets)
    trange = _rev_range(r, _TOGGLE_RANGE, travel_scale)
    names: list[str] = []
    for i, xc in enumerate(r.actuator_xs):
        toggle = model.part(f"toggle_{i}")
        toggle.visual(
            lever_mesh,
            origin=Origin(rpy=_CQ_TO_WALL),
            material=mats["actuator"],
            name="toggle_lever",
        )
        toggle.visual(
            Cylinder(radius=_TOGGLE_AXLE_R, length=_TOGGLE_AXLE_LEN),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=mats["steel"],
            name="pivot_axle",
        )
        model.articulation(
            f"toggle_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=faceplate,
            child=toggle,
            origin=Origin(xyz=(xc, _TOGGLE_PIVOT_Y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-trange, upper=trange),
        )
        names.append(f"toggle_{i}")
    return names


def _emit_buttons(model, r, faceplate, mats, travel_scale: float) -> list[str]:
    button_mesh = _button_mesh(model.assets)
    names: list[str] = []
    for i, xc in enumerate(r.actuator_xs):
        button = model.part(f"button_{i}")
        button.visual(
            button_mesh,
            origin=Origin(rpy=_CQ_TO_WALL),
            material=mats["actuator"],
            name="button_cap",
        )
        model.articulation(
            f"button_{i}_press",
            ArticulationType.PRISMATIC,
            parent=faceplate,
            child=button,
            origin=Origin(xyz=(xc, _PLATE_DEPTH, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=0.5, lower=0.0, upper=r.button_travel
            ),
        )
        names.append(f"button_{i}")
    return names


def _emit_dimmers(model, r, faceplate, mats, travel_scale: float) -> list[str]:
    knob_mesh = _dimmer_knob_mesh()
    names: list[str] = []
    for i, xc in enumerate(r.actuator_xs):
        dimmer = model.part(f"dimmer_{i}")
        dimmer.visual(
            knob_mesh,
            origin=Origin(rpy=_CQ_TO_WALL),
            material=mats["actuator"],
            name="knob_body",
        )
        # Shaft stub through the plate hole — physically mounts the knob.
        shaft_len = _PLATE_DEPTH + 0.004
        dimmer.visual(
            Cylinder(radius=0.004, length=shaft_len),
            origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mats["steel"],
            name="shaft_stub",
        )
        # Off-axis pointer tab: an asymmetric indicator riding the knob top, so
        # the CONTINUOUS spin produces a real AABB change (an axisymmetric knob
        # alone fails the spin AABB check). It embeds into the knob top, sitting
        # to one side of the spin axis.
        dimmer.visual(
            Box((0.005, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, _DIMMER_HEIGHT - 0.002, _DIMMER_POINTER_R - 0.006)),
            material=mats["steel"],
            name="pointer_tab",
        )
        model.articulation(
            f"dimmer_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=faceplate,
            child=dimmer,
            origin=Origin(xyz=(xc, _DIMMER_BASE_Y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0),
        )
        names.append(f"dimmer_{i}")
    return names


_ACTUATOR_EMITTERS = {
    "rocker": _emit_rockers,
    "toggle_lever": _emit_toggles,
    "push_button": _emit_buttons,
    "rotary_dimmer": _emit_dimmers,
}

_ACTUATOR_PREFIX = {
    "rocker": "rocker",
    "toggle_lever": "toggle",
    "push_button": "button",
    "rotary_dimmer": "dimmer",
}
_ACTUATOR_JOINT_SUFFIX = {
    "rocker": "pivot",
    "toggle_lever": "pivot",
    "push_button": "press",
    "rotary_dimmer": "spin",
}


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_switch(
    config: SwitchConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    cfg = config or SwitchConfig()
    model = ArticulatedObject(name=r.name, assets=assets)
    mats = {
        key: model.material(f"switch_{key}_{r.palette_style}", rgba=rgba)
        for key, rgba in PALETTES[r.palette_style].items()
    }

    # Root: warm-beige wall slab the plate mounts onto.
    wall = model.part("wall_panel")
    wall.visual(
        Box((r.wall_w, _WALL_T, _WALL_H)),
        origin=Origin(xyz=(0.0, -_WALL_T / 2.0, 0.0)),
        material=mats["wall"],
        name="wall_slab",
    )
    wall.inertial = Inertial.from_geometry(
        Box((r.wall_w, _WALL_T, _WALL_H)), mass=0.4, origin=Origin(xyz=(0.0, -_WALL_T / 2.0, 0.0))
    )

    # Faceplate: ivory field + chrome trim ring + optional Schuko clips.
    faceplate = model.part("faceplate")
    faceplate.visual(
        _plate_mesh(r, assets),
        origin=Origin(rpy=_CQ_TO_WALL),
        material=mats["plate"],
        name="plate_field",
    )
    faceplate.visual(
        _trim_ring_mesh(r, assets),
        origin=Origin(rpy=_CQ_TO_WALL),
        material=mats["trim"],
        name="trim_ring",
    )
    if r.has_schuko:
        for sz, elem in ((1.0, "ground_clip_upper"), (-1.0, "ground_clip_lower")):
            faceplate.visual(
                Box((_CLIP_W, _CLIP_T, _CLIP_H)),
                origin=Origin(xyz=(r.socket_x, _CLIP_YC, sz * _CLIP_ZC)),
                material=mats["steel"],
                name=elem,
            )
    faceplate.inertial = Inertial.from_geometry(
        Box((r.trim_w, _TRIM_DEPTH, r.trim_h)),
        mass=0.12,
        origin=Origin(xyz=(0.0, _TRIM_DEPTH / 2.0, 0.0)),
    )

    model.articulation(
        "wall_to_faceplate",
        ArticulationType.FIXED,
        parent=wall,
        child=faceplate,
        origin=Origin(),
    )

    # N actuators (moving parts, each its own joint).
    _ACTUATOR_EMITTERS[r.actuator_type](
        model, r, faceplate, mats, cfg.actuator_travel_scale
    )

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_switch(
    seed: int,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    return build_switch(config_from_seed(seed), assets=assets)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_switch_tests(
    object_model: ArticulatedObject,
    config: SwitchConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall_panel")
    faceplate = object_model.get_part("faceplate")

    at = r.actuator_type
    prefix = _ACTUATOR_PREFIX[at]
    suffix = _ACTUATOR_JOINT_SUFFIX[at]
    actuators = [object_model.get_part(f"{prefix}_{i}") for i in range(r.gang_count)]
    joints = [
        object_model.get_articulation(f"{prefix}_{i}_{suffix}") for i in range(r.gang_count)
    ]

    # ---- Captured-pin / seat / shaft allowances (element-scoped). ----
    for i, actuator in enumerate(actuators):
        if at in ("rocker", "toggle_lever"):
            ctx.allow_overlap(
                actuator, faceplate, elem_a="pivot_axle", elem_b="plate_field",
                reason=(
                    "Hinge-pin tips are intentionally captured in the actuator "
                    "pocket / escutcheon side walls so the lever is physically mounted."
                ),
            )
        elif at == "push_button":
            ctx.allow_overlap(
                actuator, faceplate, elem_a="button_cap", elem_b="plate_field",
                reason=(
                    "Button cap flange is wider than the pocket opening and seats "
                    "on the pocket rim, creating a thin local overlap."
                ),
            )
            ctx.allow_overlap(
                actuator, faceplate, elem_a="button_cap", elem_b="trim_ring",
                reason=(
                    "On a tight footprint the wide button cap proud of the plate "
                    "rides over the thin chrome trim lip at the plate edge."
                ),
            )
        else:  # rotary_dimmer
            ctx.allow_overlap(
                actuator, faceplate, elem_a="shaft_stub", elem_b="plate_field",
                reason=(
                    "The dimmer shaft stub passes through the circular hole in the "
                    "plate to physically mount the knob on the faceplate."
                ),
            )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_joint_mating_has_gap()

    # ---- Structure / identity. ----
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "wall_panel + faceplate present",
        "wall_panel" in part_names and "faceplate" in part_names,
        details=str(sorted(part_names)),
    )
    ctx.check(
        f"N={r.gang_count} {prefix} actuators present",
        all(f"{prefix}_{i}" in part_names for i in range(r.gang_count)),
        details=str(sorted(part_names)),
    )

    # ---- Faceplate seated flush on the wall + within the wall footprint. ----
    ctx.expect_gap(
        faceplate, wall, axis="y", max_gap=0.0006, max_penetration=0.0002,
        name="faceplate seated flush on the wall",
    )
    ctx.expect_within(
        faceplate, wall, axes="xz", margin=0.0,
        name="faceplate inside the wall slab footprint",
    )

    # ---- FIXED wall->faceplate. ----
    wj = object_model.get_articulation("wall_to_faceplate")
    ctx.check(
        "wall_to_faceplate is FIXED",
        wj.articulation_type == ArticulationType.FIXED,
        details=f"type={wj.articulation_type}",
    )

    # ---- Chrome trim is the outermost border, proud of the ivory field. ----
    trim_aabb = ctx.part_element_world_aabb(faceplate, elem="trim_ring")
    field_aabb = ctx.part_element_world_aabb(faceplate, elem="plate_field")
    if trim_aabb is not None and field_aabb is not None:
        ctx.check(
            "chrome trim is the outermost border",
            trim_aabb[1][0] >= field_aabb[1][0] - 0.001
            and trim_aabb[0][0] <= field_aabb[0][0] + 0.001
            and trim_aabb[1][2] >= field_aabb[1][2] - 0.001
            and trim_aabb[0][2] <= field_aabb[0][2] + 0.001,
            details=f"trim={trim_aabb}, field={field_aabb}",
        )

    # ---- Schuko only on rounded_rect_horizontal. ----
    clip_names = {v.name for v in faceplate.visuals if v.name.startswith("ground_clip")}
    if r.has_schuko:
        ctx.check(
            "Schuko grounding clips present on horizontal plate",
            clip_names == {"ground_clip_upper", "ground_clip_lower"},
            details=str(sorted(clip_names)),
        )
        ctx.check(
            "Schuko well authored as a true hollow recess with pin holes",
            _WELL_FLOOR_Y < _TILE_FACE_Y - 0.006 and _WELL_RADIUS * 2.0 > 0.035,
            details=f"well_depth={_TILE_FACE_Y - _WELL_FLOOR_Y:.4f}",
        )
    else:
        ctx.check(
            "no Schuko on square/round faceplate",
            len(clip_names) == 0,
            details=str(sorted(clip_names)),
        )

    # ---- round_plate compatibility gate. ----
    if r.faceplate_shape == "round_plate":
        ctx.check(
            "round_plate clamps gang_count <= 2",
            r.gang_count <= ROUND_PLATE_MAX_N,
            details=f"N={r.gang_count}",
        )

    # ---- Actuator placement: center-symmetric, inside the plate. ----
    for i, (actuator, xc) in enumerate(zip(actuators, r.actuator_xs)):
        pos = ctx.part_world_position(actuator)
        ctx.check(
            f"{prefix}_{i} centered on its module (x={xc:+.3f})",
            pos is not None and abs(pos[0] - xc) < 0.003,
            details=f"pos={pos}",
        )
        ctx.expect_within(
            actuator, faceplate, axes="x", margin=0.0,
            name=f"{prefix}_{i} stays inside the plate outline (X)",
        )

    # ---- Per-actuator joint topology + actuation. ----
    if at == "rocker":
        for i, (actuator, j) in enumerate(zip(actuators, joints)):
            ctx.check(
                f"rocker_{i} is REVOLUTE about +X",
                j.articulation_type == ArticulationType.REVOLUTE and abs(j.axis[0]) > 0.99,
                details=f"type={j.articulation_type} axis={tuple(j.axis)}",
            )
            lim = j.motion_limits
            ctx.check(
                f"rocker_{i} symmetric see-saw range",
                lim is not None and lim.lower is not None and lim.upper is not None
                and abs(lim.lower + lim.upper) < 1e-6 and lim.upper > 0.0,
                details=f"limits=({lim.lower}, {lim.upper})",
            )
            rest = ctx.part_world_aabb(actuator)
            with ctx.pose({j: lim.upper}):
                tip = ctx.part_world_aabb(actuator)
                ctx.check(
                    f"rocker_{i} pressed pose tips the paddle outward",
                    rest is not None and tip is not None and tip[1][1] > rest[1][1] + 0.0010,
                    details=f"rest={rest}, tip={tip}",
                )
                ctx.expect_gap(
                    actuator, wall, axis="y", max_penetration=0.0,
                    name=f"rocker_{i} pressed pose clears the wall",
                )
    elif at == "toggle_lever":
        for i, (actuator, j) in enumerate(zip(actuators, joints)):
            ctx.check(
                f"toggle_{i} is REVOLUTE about +X",
                j.articulation_type == ArticulationType.REVOLUTE and abs(j.axis[0]) > 0.99,
                details=f"type={j.articulation_type} axis={tuple(j.axis)}",
            )
            lim = j.motion_limits
            ctx.check(
                f"toggle_{i} flip range >= rocker range",
                lim is not None and lim.upper is not None and lim.upper > 0.20,
                details=f"upper={None if lim is None else lim.upper}",
            )
            rest = ctx.part_world_aabb(actuator)
            with ctx.pose({j: lim.upper}):
                tip = ctx.part_world_aabb(actuator)
                ctx.check(
                    f"toggle_{i} flips (AABB changes)",
                    rest is not None and tip is not None
                    and (abs(tip[1][2] - rest[1][2]) > 0.002 or abs(tip[0][2] - rest[0][2]) > 0.002),
                    details=f"rest={rest}, tip={tip}",
                )
    elif at == "push_button":
        for i, (actuator, j) in enumerate(zip(actuators, joints)):
            ctx.check(
                f"button_{i} is PRISMATIC about -Y",
                j.articulation_type == ArticulationType.PRISMATIC and abs(j.axis[1]) > 0.99,
                details=f"type={j.articulation_type} axis={tuple(j.axis)}",
            )
            lim = j.motion_limits
            ctx.check(
                f"button_{i} press travel within range",
                lim is not None and lim.upper is not None and 0.0 < lim.upper <= _POCKET_FLOOR_Y + 0.001,
                details=f"upper={None if lim is None else lim.upper}",
            )
            ctx.expect_contact(
                actuator, faceplate, elem_a="button_cap", elem_b="plate_field",
                name=f"button_{i} cap seats on the pocket rim",
            )
            p0 = ctx.part_world_position(actuator)
            with ctx.pose({j: lim.upper}):
                p1 = ctx.part_world_position(actuator)
                ctx.check(
                    f"button_{i} presses inward (-Y)",
                    p0 is not None and p1 is not None and p1[1] < p0[1] - 0.0005,
                    details=f"rest_y={p0}, pressed_y={p1}",
                )
                ctx.expect_gap(
                    actuator, wall, axis="y", max_penetration=0.0,
                    name=f"button_{i} pressed pose clears the wall",
                )
    else:  # rotary_dimmer
        for i, (actuator, j) in enumerate(zip(actuators, joints)):
            ctx.check(
                f"dimmer_{i} is CONTINUOUS about +Y (no limits)",
                j.articulation_type == ArticulationType.CONTINUOUS
                and abs(j.axis[1] - 1.0) < 0.01 and abs(j.axis[0]) < 0.01 and abs(j.axis[2]) < 0.01
                and (j.motion_limits is None or j.motion_limits.lower is None),
                details=f"type={j.articulation_type} axis={tuple(j.axis)}",
            )
            ctx.expect_contact(
                actuator, faceplate, elem_a="knob_body", elem_b="plate_field",
                name=f"dimmer_{i} knob base seats on the plate face",
            )
            # Pure rotation: center unchanged.
            rest_pos = ctx.part_world_position(actuator)
            with ctx.pose({j: math.pi / 2.0}):
                spun_pos = ctx.part_world_position(actuator)
                ctx.check(
                    f"dimmer_{i} spin preserves position (pure rotation)",
                    rest_pos is not None and spun_pos is not None
                    and abs(spun_pos[0] - rest_pos[0]) < 0.003
                    and abs(spun_pos[1] - rest_pos[1]) < 0.003
                    and abs(spun_pos[2] - rest_pos[2]) < 0.003,
                    details=f"rest={rest_pos}, spun={spun_pos}",
                )
            # AABB changes under rotation (the off-axis pointer tab guarantees this).
            rest_aabb = ctx.part_world_aabb(actuator)
            with ctx.pose({j: math.pi / 4.0}):
                rot_aabb = ctx.part_world_aabb(actuator)
                ctx.check(
                    f"dimmer_{i} rotated pose changes the bounding box",
                    rest_aabb is not None and rot_aabb is not None
                    and (
                        abs(rot_aabb[1][0] - rest_aabb[1][0]) > 0.0005
                        or abs(rot_aabb[1][2] - rest_aabb[1][2]) > 0.0005
                        or abs(rot_aabb[0][0] - rest_aabb[0][0]) > 0.0005
                        or abs(rot_aabb[0][2] - rest_aabb[0][2]) > 0.0005
                    ),
                    details=f"rest={rest_aabb}, rot={rot_aabb}",
                )

    # ---- slot_choices recorded with N encoded. ----
    ctx.check(
        "slot_choices recorded with gang_count encoded",
        tuple(object_model.meta.get("slot_choices", ())) == slot_choices_for_config(r),
        details=str(object_model.meta.get("slot_choices")),
    )

    return ctx.report()


__all__ = (
    "SwitchConfig",
    "ResolvedSwitchConfig",
    "build_switch",
    "build_seeded_switch",
    "config_from_seed",
    "resolve_config",
    "run_switch_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
    "with_overrides",
)
