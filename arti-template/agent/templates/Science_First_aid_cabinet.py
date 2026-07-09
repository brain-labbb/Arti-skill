"""first_aid_cabinet — modular procedural template (wall-mounted first-aid cabinet).

Category identity: a white sheet-metal wall cabinet **cabinet_body** is the single
grounded root (front-open hollow shell + carry handle + red-cross / FIRST AID
identity marks, wall behind it). The front is closed by one or two REVOLUTE doors
hinged on a vertical (Z) edge; the interior is divided by FIXED horizontal
**shelf_{i}** visuals and may add a stack of +Y PRISMATIC **drawer_{i}** trays.

Frame convention (shared across every module):
  +X = right, -X = left (hinge side for single door)
  +Y = forward (out of the cabinet face); wall is at -Y
  +Z = up; the cabinet bottom rests on z = 0.

Two named slots + two multiplicity axes:
  Slot A  door            : glass_front_hinged / solid_panel_hinged / double_doors
  Slot B  interior_fitment: open_shelves / shelves_plus_drawer / shelves_plus_drawer_stack
  shelf_count   axis : FIXED shelf_{i} visuals (equispaced)
  drawer_count  axis : +Y PRISMATIC drawer_{i} (derived from fitment)

5-star module sources (all synced under data/records/, rating=5):
  Slot A door:
    glass_front_hinged  — rec_build-...-firs_..._99727092 (parent)
    solid_panel_hinged  — rec_first_aid_cabinet_var_solid_door
    double_doors        — rec_first_aid_cabinet_var_double_doors
  Slot B interior_fitment:
    open_shelves        — parent / one_shelf / three_shelf
    shelves_plus_drawer — rec_first_aid_cabinet_var_drawer_base
    shelves_plus_drawer_stack — rec_first_aid_cabinet_var_drawer_stack

Canonical spec: articraft_template_authoring/specs_modular_v1/Science_First_aid_Other_Cabinet.md
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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

__modular__ = True

# ---------------------------------------------------------------------------
# Enum domains
# ---------------------------------------------------------------------------
DoorChoice = Literal["glass_front_hinged", "solid_panel_hinged", "double_doors"]
FitmentChoice = Literal[
    "open_shelves",
    "shelves_plus_drawer",
    "shelves_plus_drawer_stack",
]
PaletteStyle = Literal[
    "clinical_white_redcross",
    "stainless_steel",
    "emergency_green",
    "industrial_grey",
    "vintage_enamel",
]

DOOR_CHOICES: tuple[DoorChoice, ...] = (
    "glass_front_hinged",
    "solid_panel_hinged",
    "double_doors",
)
FITMENT_CHOICES: tuple[FitmentChoice, ...] = (
    "open_shelves",
    "shelves_plus_drawer",
    "shelves_plus_drawer_stack",
)
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "clinical_white_redcross",
    "stainless_steel",
    "emergency_green",
    "industrial_grey",
    "vintage_enamel",
)

# shelf_count product domain [1,6]; small N high frequency.
SHELF_COUNTS = (1, 2, 3, 4, 5, 6)
SHELF_COUNT_WEIGHTS = (26, 30, 24, 10, 6, 4)
# drawer_count for the stack fitment [2,4]; small N more frequent.
STACK_DRAWER_COUNTS = (2, 3, 4)
STACK_DRAWER_WEIGHTS = (40, 35, 25)

# ---------------------------------------------------------------------------
# Palette table — body / front / glass / cross+banner / hardware
# (5 styles; per-seed sampled; only material rgba changes, never topology).
# ---------------------------------------------------------------------------
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "clinical_white_redcross": {
        "body": (0.92, 0.92, 0.93, 1.0),
        "front": (0.95, 0.95, 0.96, 1.0),
        "glass": (0.62, 0.74, 0.80, 0.45),
        "cross": (0.78, 0.10, 0.12, 1.0),
        "hardware": (0.70, 0.72, 0.75, 1.0),
        "shelf": (0.85, 0.86, 0.87, 1.0),
        "supply_a": (0.20, 0.40, 0.72, 1.0),
        "supply_b": (0.86, 0.80, 0.66, 1.0),
    },
    "stainless_steel": {
        "body": (0.74, 0.75, 0.77, 1.0),
        "front": (0.78, 0.79, 0.81, 1.0),
        "glass": (0.55, 0.60, 0.64, 0.45),
        "cross": (0.78, 0.10, 0.12, 1.0),
        "hardware": (0.55, 0.57, 0.60, 1.0),
        "shelf": (0.70, 0.72, 0.74, 1.0),
        "supply_a": (0.22, 0.42, 0.70, 1.0),
        "supply_b": (0.82, 0.78, 0.70, 1.0),
    },
    "emergency_green": {
        "body": (0.16, 0.45, 0.30, 1.0),
        "front": (0.18, 0.50, 0.33, 1.0),
        "glass": (0.62, 0.74, 0.80, 0.45),
        "cross": (0.97, 0.97, 0.97, 1.0),
        "hardware": (0.70, 0.72, 0.75, 1.0),
        "shelf": (0.80, 0.84, 0.80, 1.0),
        "supply_a": (0.90, 0.90, 0.90, 1.0),
        "supply_b": (0.86, 0.80, 0.66, 1.0),
    },
    "industrial_grey": {
        "body": (0.55, 0.56, 0.58, 1.0),
        "front": (0.58, 0.59, 0.61, 1.0),
        "glass": (0.50, 0.54, 0.58, 0.45),
        "cross": (0.78, 0.10, 0.12, 1.0),
        "hardware": (0.45, 0.47, 0.50, 1.0),
        "shelf": (0.66, 0.67, 0.69, 1.0),
        "supply_a": (0.24, 0.40, 0.66, 1.0),
        "supply_b": (0.80, 0.76, 0.64, 1.0),
    },
    "vintage_enamel": {
        "body": (0.90, 0.88, 0.82, 1.0),
        "front": (0.92, 0.90, 0.84, 1.0),
        "glass": (0.66, 0.72, 0.70, 0.45),
        "cross": (0.62, 0.10, 0.12, 1.0),
        "hardware": (0.72, 0.60, 0.32, 1.0),
        "shelf": (0.84, 0.82, 0.76, 1.0),
        "supply_a": (0.30, 0.46, 0.62, 1.0),
        "supply_b": (0.84, 0.78, 0.62, 1.0),
    },
}


# ---------------------------------------------------------------------------
# Config dataclasses
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class FirstAidCabinetConfig:
    door_choice: DoorChoice = "glass_front_hinged"
    fitment_choice: FitmentChoice = "open_shelves"
    shelf_count: int = 2
    drawer_count: int = 0  # derived from fitment in resolve_config
    palette_style: PaletteStyle = "clinical_white_redcross"
    body_width_scale: float = 1.0
    body_height_scale: float = 1.0
    door_thickness_scale: float = 1.0
    drawer_travel_scale: float = 1.0
    name: str = "reference_first_aid_cabinet"


@dataclass(frozen=True)
class ResolvedFirstAidCabinetConfig:
    door_choice: DoorChoice
    fitment_choice: FitmentChoice
    shelf_count: int
    drawer_count: int
    palette_style: PaletteStyle
    # Envelope (meters)
    body_w: float
    body_d: float
    body_h: float
    wall_t: float
    door_t: float
    drawer_travel: float
    name: str


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# Procedural sampler
# ---------------------------------------------------------------------------
def config_from_seed(seed: int) -> FirstAidCabinetConfig:
    rng = random.Random(seed)
    door = rng.choice(DOOR_CHOICES)
    fitment = rng.choice(FITMENT_CHOICES)

    # drawer_count is conditional on fitment.
    if fitment == "open_shelves":
        drawer_count = 0
    elif fitment == "shelves_plus_drawer":
        drawer_count = 1
    else:  # shelves_plus_drawer_stack
        drawer_count = rng.choices(STACK_DRAWER_COUNTS, weights=STACK_DRAWER_WEIGHTS, k=1)[0]

    shelf_count = rng.choices(SHELF_COUNTS, weights=SHELF_COUNT_WEIGHTS, k=1)[0]

    return FirstAidCabinetConfig(
        door_choice=door,
        fitment_choice=fitment,
        shelf_count=shelf_count,
        drawer_count=drawer_count,
        palette_style=rng.choice(PALETTE_STYLES),
        body_width_scale=round(rng.uniform(0.85, 1.20), 3),
        body_height_scale=round(rng.uniform(0.85, 1.25), 3),
        door_thickness_scale=round(rng.uniform(0.85, 1.15), 3),
        drawer_travel_scale=round(rng.uniform(0.85, 1.15), 3),
        name=f"seeded_first_aid_cabinet_{seed}",
    )


def _max_shelf_count_upper(body_h: float, wall_t: float, fitment: FitmentChoice) -> int:
    """Inequality projection: shelf rows must fit within their usable zone.

    In stack mode shelves live only in the upper half (above SPLIT_Z); otherwise
    they span the full inner cavity. Each shelf needs >= ~0.030 m clearance.
    """
    inner_h = body_h - 2.0 * wall_t
    zone_h = inner_h / 2.0 if fitment == "shelves_plus_drawer_stack" else inner_h
    # Equispaced into (N+1) cells; require each cell >= 0.030 m.
    max_n = int(zone_h / 0.030) - 1
    return max(1, min(6, max_n))


def resolve_config(config: FirstAidCabinetConfig) -> ResolvedFirstAidCabinetConfig:
    if config.door_choice not in DOOR_CHOICES:
        raise ValueError(f"Unsupported door_choice: {config.door_choice}")
    if config.fitment_choice not in FITMENT_CHOICES:
        raise ValueError(f"Unsupported fitment_choice: {config.fitment_choice}")
    if config.palette_style not in PALETTES:
        raise ValueError(f"Unsupported palette_style: {config.palette_style}")

    fitment = config.fitment_choice

    w_scale = _clamp(config.body_width_scale, 0.85, 1.20)
    h_scale = _clamp(config.body_height_scale, 0.85, 1.25)
    t_scale = _clamp(config.door_thickness_scale, 0.85, 1.15)
    travel_scale = _clamp(config.drawer_travel_scale, 0.85, 1.15)

    wall_t = 0.010
    body_w = round(0.340 * w_scale, 4)
    body_d = 0.130
    body_h = round(0.400 * h_scale, 4)
    door_t = round(0.022 * t_scale, 4)

    # Conditional drawer_count domain (mutually-exclusive gate by fitment).
    if fitment == "open_shelves":
        drawer_count = 0
    elif fitment == "shelves_plus_drawer":
        drawer_count = 1
    else:  # shelves_plus_drawer_stack
        drawer_count = config.drawer_count if config.drawer_count in STACK_DRAWER_COUNTS else 3

    # shelf_count clamped to its usable zone via the inequality projection.
    shelf_cap = _max_shelf_count_upper(body_h, wall_t, fitment)
    shelf_count = config.shelf_count if config.shelf_count in SHELF_COUNTS else 2
    shelf_count = max(1, min(shelf_count, shelf_cap))

    # Prismatic travel (single drawer ~0.080, stack ~0.085).
    base_travel = 0.080 if fitment == "shelves_plus_drawer" else 0.085
    drawer_travel = round(base_travel * travel_scale, 4)

    return ResolvedFirstAidCabinetConfig(
        door_choice=config.door_choice,
        fitment_choice=fitment,
        shelf_count=shelf_count,
        drawer_count=drawer_count,
        palette_style=config.palette_style,
        body_w=body_w,
        body_d=body_d,
        body_h=body_h,
        wall_t=wall_t,
        door_t=door_t,
        drawer_travel=drawer_travel,
        name=config.name or "first_aid_cabinet",
    )


# ---------------------------------------------------------------------------
# slot_choices
# ---------------------------------------------------------------------------
def _slot_choices_for_resolved(r: ResolvedFirstAidCabinetConfig) -> list[tuple[str, str]]:
    return [
        ("door", r.door_choice),
        ("interior_fitment", r.fitment_choice),
        ("shelf_count", f"x{r.shelf_count}"),
        ("drawer_count", f"x{r.drawer_count}"),
    ]


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return _slot_choices_for_resolved(resolve_config(config_from_seed(seed)))


# ---------------------------------------------------------------------------
# Geometry frame — every derived dimension lives here.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class _Frame:
    BODY_W: float
    BODY_D: float
    BODY_H: float
    WALL_T: float
    DOOR_T: float
    fitment: FitmentChoice

    @property
    def INNER_W(self) -> float:
        return self.BODY_W - 2 * self.WALL_T

    @property
    def INNER_H(self) -> float:
        return self.BODY_H - 2 * self.WALL_T

    @property
    def INNER_D(self) -> float:
        return self.BODY_D - self.WALL_T

    @property
    def Z_LIFT(self) -> float:
        return self.BODY_H / 2.0

    @property
    def SPLIT_Z(self) -> float:
        # body-local z dividing upper (door) from lower (drawer stack)
        return 0.0

    @property
    def UPPER_H(self) -> float:
        return self.INNER_H / 2.0

    @property
    def LOWER_H(self) -> float:
        return self.INNER_H / 2.0


def _make_frame(r: ResolvedFirstAidCabinetConfig) -> _Frame:
    return _Frame(
        BODY_W=r.body_w,
        BODY_D=r.body_d,
        BODY_H=r.body_h,
        WALL_T=r.wall_t,
        DOOR_T=r.door_t,
        fitment=r.fitment_choice,
    )


SHELF_T = 0.008
SHELF_EMBED = 0.003
HINGE_INSET = 0.006
HINGE_R = 0.006


def _shelf_y_geom(fr: _Frame) -> tuple[float, float, float]:
    """Return (depth, center_y) for a shelf that tucks into back wall, clears
    the closed door (its front edge stays behind the seated door panel)."""
    back = -fr.BODY_D / 2.0 + fr.WALL_T - SHELF_EMBED
    front = fr.BODY_D / 2.0 - fr.DOOR_T - 0.004
    front = max(front, back + 0.040)
    return (front - back, (front + back) / 2.0)


def _shelf_zs(fr: _Frame, n: int) -> list[float]:
    """Equispaced shelf z-centers (body-local). Stack mode → upper half only."""
    if fr.fitment == "shelves_plus_drawer_stack":
        zone_bot = fr.SPLIT_Z + fr.WALL_T / 2.0
        zone_top = fr.INNER_H / 2.0
    else:
        zone_bot = -fr.INNER_H / 2.0
        zone_top = fr.INNER_H / 2.0
    span = zone_top - zone_bot
    return [zone_bot + (i + 1) * span / (n + 1) for i in range(n)]


def _drawer_slot_z(fr: _Frame, i: int, n: int) -> float:
    """Body-local z center of drawer slot i (lower half, equispaced)."""
    slot_h = fr.LOWER_H / n
    return -fr.INNER_H / 2.0 + slot_h * (i + 0.5)


# ---------------------------------------------------------------------------
# CadQuery geometry builders (preserve source primitives: cut/fillet windows)
# ---------------------------------------------------------------------------
def _build_body_shell_cq(fr: _Frame, n_drawers: int) -> cq.Workplane:
    """Hollow white cabinet box, open on the +Y (front) face.

    In stack mode, add a horizontal divider at SPLIT_Z and a lower front panel
    with n_drawers milled openings (source: drawer_stack build_body_shell)."""
    outer = cq.Workplane("XY").box(fr.BODY_W, fr.BODY_D, fr.BODY_H).edges("|Z").fillet(0.004)
    cavity = (
        cq.Workplane("XY")
        .box(fr.INNER_W, fr.BODY_D, fr.INNER_H)
        .translate((0.0, fr.WALL_T / 2.0 + 0.001, 0.0))
    )
    shell = outer.cut(cavity)

    if fr.fitment == "shelves_plus_drawer_stack":
        divider = (
            cq.Workplane("XY")
            .box(fr.INNER_W + 0.002, fr.INNER_D + 0.002, fr.WALL_T)
            .translate((0.0, fr.WALL_T / 2.0, fr.SPLIT_Z))
        )
        shell = shell.union(divider)

        lower_open_bottom = -fr.INNER_H / 2.0
        lower_open_top = fr.SPLIT_Z - fr.WALL_T / 2.0
        lower_panel_h = lower_open_top - lower_open_bottom
        lower_panel_z = (lower_open_bottom + lower_open_top) / 2.0
        lower_panel = (
            cq.Workplane("XY")
            .box(fr.INNER_W + 0.002, fr.WALL_T, lower_panel_h)
            .translate((0.0, fr.BODY_D / 2.0 - fr.WALL_T / 2.0, lower_panel_z))
        )
        slot_h = fr.LOWER_H / n_drawers
        open_h = slot_h - 0.004
        open_w = fr.INNER_W - 0.010
        for i in range(n_drawers):
            slot_z = _drawer_slot_z(fr, i, n_drawers)
            opening = (
                cq.Workplane("XY")
                .box(open_w, fr.WALL_T + 0.010, open_h)
                .translate((0.0, fr.BODY_D / 2.0 - fr.WALL_T / 2.0, slot_z))
            )
            lower_panel = lower_panel.cut(opening)
        shell = shell.union(lower_panel)
    elif fr.fitment == "shelves_plus_drawer":
        # Bottom front panel with a single drawer opening (source: drawer_base).
        lower_open_bottom = -fr.INNER_H / 2.0
        lower_open_top = -fr.INNER_H / 2.0 + fr.LOWER_H * 0.55
        lower_panel_h = lower_open_top - lower_open_bottom
        lower_panel_z = (lower_open_bottom + lower_open_top) / 2.0
        lower_panel = (
            cq.Workplane("XY")
            .box(fr.INNER_W + 0.002, fr.WALL_T, lower_panel_h)
            .translate((0.0, fr.BODY_D / 2.0 - fr.WALL_T / 2.0, lower_panel_z))
        )
        open_w = 0.260 * (fr.BODY_W / 0.340)
        open_w = min(open_w, fr.INNER_W - 0.010)
        open_h = lower_panel_h - 0.012
        opening = (
            cq.Workplane("XY")
            .box(open_w, fr.WALL_T + 0.010, open_h)
            .translate((0.0, fr.BODY_D / 2.0 - fr.WALL_T / 2.0, lower_panel_z))
        )
        lower_panel = lower_panel.cut(opening)
        shell = shell.union(lower_panel)
    return shell


def _build_shelf_cq(fr: _Frame) -> cq.Workplane:
    depth, _ = _shelf_y_geom(fr)
    return cq.Workplane("XY").box(fr.INNER_W + 2 * SHELF_EMBED, depth, SHELF_T)


def _build_supply_cq(w: float, d: float, h: float) -> cq.Workplane:
    return cq.Workplane("XY").box(w, d, h).edges("|Z").fillet(0.002)


def _build_handle_cq() -> cq.Workplane:
    grip_len, riser_h, bar_t, bar_w = 0.150, 0.022, 0.006, 0.014
    grip = cq.Workplane("XY").box(grip_len, bar_w, bar_t).translate((0.0, 0.0, riser_h))
    riser_l = (
        cq.Workplane("XY")
        .box(bar_t, bar_w, riser_h)
        .translate((-grip_len / 2.0 + bar_t / 2.0, 0.0, riser_h / 2.0))
    )
    riser_r = (
        cq.Workplane("XY")
        .box(bar_t, bar_w, riser_h)
        .translate((grip_len / 2.0 - bar_t / 2.0, 0.0, riser_h / 2.0))
    )
    return grip.union(riser_l).union(riser_r).edges("|Y").fillet(0.0025)


def _build_hinge_knuckles_cq(z_stations: list[float], knuckle_h: float) -> cq.Workplane:
    barrels = None
    for z in z_stations:
        seg = cq.Workplane("XY").cylinder(knuckle_h, HINGE_R).translate((0.0, 0.0, z))
        barrels = seg if barrels is None else barrels.union(seg)
    return barrels


# Door geometry (leaf authored in its own local frame; hinge line at local x=0).
def _build_door_frame_cq(door_w: float, door_h: float, door_t: float, frame_b: float) -> cq.Workplane:
    cx = HINGE_R + door_w / 2.0
    panel = (
        cq.Workplane("XY")
        .box(door_w, door_t, door_h)
        .translate((cx, 0.0, 0.0))
        .edges("|Z").fillet(0.003)
    )
    win_w = door_w - 2 * frame_b
    win_h = door_h - 2 * frame_b
    window_cut = (
        cq.Workplane("XY")
        .box(win_w, door_t + 0.01, win_h)
        .translate((cx, 0.0, 0.0))
    )
    return panel.cut(window_cut)


def _build_door_panel_cq(door_w: float, door_h: float, door_t: float) -> cq.Workplane:
    """Solid sheet-metal door panel (no window) — source: solid_door."""
    cx = HINGE_R + door_w / 2.0
    return (
        cq.Workplane("XY")
        .box(door_w, door_t, door_h)
        .translate((cx, 0.0, 0.0))
        .edges("|Z").fillet(0.003)
    )


def _build_door_glass_cq(door_w: float, door_h: float, door_t: float, frame_b: float) -> cq.Workplane:
    cx = HINGE_R + door_w / 2.0
    win_w = door_w - 2 * frame_b
    win_h = door_h - 2 * frame_b
    return (
        cq.Workplane("XY")
        .box(win_w + 0.006, 0.004, win_h + 0.006)
        .translate((cx, 0.0, 0.0))
    )


def _build_door_emblem_cq(door_w: float, door_t: float, on_glass: bool) -> cq.Workplane:
    arm, bar, t = 0.025, 0.020, 0.0015
    cx = HINGE_R + door_w / 2.0
    vert = cq.Workplane("XY").box(bar, t, 2 * arm)
    horiz = cq.Workplane("XY").box(2 * arm, t, bar)
    cross = vert.union(horiz)
    # On glass: bonded onto glass front face (y=+0.002). On solid: printed on
    # panel front face (y=DOOR_T/2).
    front_y = 0.002 if on_glass else door_t / 2.0
    return cross.translate((cx, front_y, 0.0))


def _build_door_banner_cq(door_w: float, door_h: float, door_t: float, frame_b: float) -> cq.Workplane:
    cx = HINGE_R + door_w / 2.0
    band_h = max(frame_b - 0.010, 0.008)
    return (
        cq.Workplane("XY")
        .box(door_w - 0.012, 0.0020, band_h)
        .translate((cx, door_t / 2.0, door_h / 2.0 - band_h / 2.0 - 0.004))
    )


def _build_door_knob_cq(door_w: float, door_t: float, frame_b: float, free_edge_sign: float) -> cq.Workplane:
    knob = cq.Workplane("XY").cylinder(0.018, 0.007)
    knob = knob.rotate((0, 0, 0), (1, 0, 0), 90)
    # Free edge: for the left-hinged single door the free edge is at large +x;
    # for a right-hinged leaf it's near the hinge — caller passes sign.
    knob_x = HINGE_R + door_w - frame_b / 2.0 if free_edge_sign > 0 else HINGE_R + frame_b / 2.0
    return knob.translate((knob_x, door_t / 2.0 + 0.006, 0.0))


# Drawer tray (shared helper) — open-top tray + front face + pull.
def _build_drawer_tray_cq(fr: _Frame, drawer_w: float, box_h: float, open_w: float) -> cq.Workplane:
    """Drawer authored in its part frame: front face outer at local y=0, tray
    extends back along -Y (source: drawer_base/drawer_stack build_drawer_tray)."""
    w = drawer_w
    d = fr.INNER_D - 0.014
    h = box_h
    t = 0.003
    ft = 0.005
    front_w = open_w
    front_h = h + 0.004
    front = cq.Workplane("XY").box(front_w, ft, front_h)
    bottom = (
        cq.Workplane("XY")
        .box(w, d, t)
        .translate((0.0, -d / 2.0 - ft / 2.0, -h / 2.0 + t / 2.0))
    )
    back = (
        cq.Workplane("XY")
        .box(w, t, h)
        .translate((0.0, -d - ft / 2.0 + t / 2.0, 0.0))
    )
    left = (
        cq.Workplane("XY")
        .box(t, d, h)
        .translate((-w / 2.0 + t / 2.0, -d / 2.0 - ft / 2.0, 0.0))
    )
    right = (
        cq.Workplane("XY")
        .box(t, d, h)
        .translate((w / 2.0 - t / 2.0, -d / 2.0 - ft / 2.0, 0.0))
    )
    pull = (
        cq.Workplane("XY")
        .box(0.060, 0.010, 0.010)
        .translate((0.0, ft / 2.0 + 0.005, 0.0))
    )
    return front.union(bottom).union(back).union(left).union(right).union(pull)


# ---------------------------------------------------------------------------
# Carcass (root) — body shell + handle + shelves + supplies + hinge barrels.
# Returns list of element-scoped allow_overlap specs.
# ---------------------------------------------------------------------------
def _build_carcass(model, fr: _Frame, r, mats) -> list[tuple]:
    overlaps: list[tuple] = []
    body = model.part("cabinet_body")
    body.visual(
        mesh_from_cadquery(_build_body_shell_cq(fr, max(r.drawer_count, 1)), "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, fr.Z_LIFT)),
        material=mats["body"],
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((fr.BODY_W, fr.BODY_D, fr.BODY_H)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, fr.Z_LIFT)),
    )

    # Carry handle on top.
    body.visual(
        mesh_from_cadquery(_build_handle_cq(), "handle"),
        origin=Origin(xyz=(0.0, 0.0, fr.BODY_H / 2.0 + fr.Z_LIFT)),
        material=mats["hardware"],
        name="carry_handle",
    )

    # Interior FIXED shelves (shelf_count axis).
    _, shelf_y = _shelf_y_geom(fr)
    shelf_mesh = mesh_from_cadquery(_build_shelf_cq(fr), "shelf")
    shelf_zs = _shelf_zs(fr, r.shelf_count)
    for i, z in enumerate(shelf_zs):
        body.visual(
            shelf_mesh,
            origin=Origin(xyz=(0.0, shelf_y, z + fr.Z_LIFT)),
            material=mats["shelf"],
            name=f"shelf_{i}",
        )
        overlaps.append(
            ("cabinet_body", "cabinet_body", f"shelf_{i}", "body_shell",
             "Shelf is welded into the side and back walls (seated shelf).")
        )

    # Supplies seated on the lowest shelf (always present, solid surface).
    # Sized to the available width/height so they never poke through walls/door.
    shelf0_top = shelf_zs[0] + SHELF_T / 2.0
    supply_h_a = min(0.055, fr.INNER_W * 0.16 + 0.020)
    supply_w = min(0.080, fr.INNER_W / 2.0 - 0.012)
    supply_d = 0.050
    supply_a = mesh_from_cadquery(_build_supply_cq(supply_w, supply_d, supply_h_a), "supply_a")
    supply_b = mesh_from_cadquery(_build_supply_cq(supply_w * 0.8, supply_d, supply_h_a * 0.9), "supply_b")
    supply_y = fr.WALL_T / 2.0 + 0.012
    embed = 0.003
    body.visual(
        supply_a,
        origin=Origin(xyz=(-supply_w / 2.0 - 0.008, supply_y, shelf0_top + supply_h_a / 2.0 - embed + fr.Z_LIFT)),
        material=mats["supply_a"],
        name="supply_floor_l",
    )
    body.visual(
        supply_b,
        origin=Origin(xyz=(supply_w / 2.0 + 0.008, supply_y, shelf0_top + supply_h_a * 0.9 / 2.0 - embed + fr.Z_LIFT)),
        material=mats["supply_b"],
        name="supply_floor_r",
    )
    overlaps.append(("cabinet_body", "cabinet_body", "supply_floor_l", "shelf_0",
                     "Supply box rests seated on the lowest shelf (small embed)."))
    overlaps.append(("cabinet_body", "cabinet_body", "supply_floor_r", "shelf_0",
                     "Supply box rests seated on the lowest shelf (small embed)."))
    return overlaps


# ---------------------------------------------------------------------------
# Door modules — return (joint_names, overlaps).
# ---------------------------------------------------------------------------
def _hinge_z_body(fr: _Frame) -> float:
    # Door spans the whole cavity except in stack mode (upper half only).
    if fr.fitment == "shelves_plus_drawer_stack":
        return fr.SPLIT_Z + fr.UPPER_H / 2.0
    return 0.0


def _door_region_h(fr: _Frame) -> float:
    if fr.fitment == "shelves_plus_drawer_stack":
        return fr.UPPER_H - 0.005
    if fr.fitment == "shelves_plus_drawer":
        # door covers from top down to the bottom drawer panel
        return fr.INNER_H - fr.LOWER_H * 0.55 - 0.005
    return fr.BODY_H - 0.010


def _build_single_door(model, fr: _Frame, r, mats, *, glass: bool):
    """glass_front_hinged / solid_panel_hinged — one cabinet_door leaf, REVOLUTE +Z."""
    overlaps: list[tuple] = []
    door_h = _door_region_h(fr)
    door_w = min(fr.BODY_W - 0.010, fr.BODY_W * 0.97)
    door_t = fr.DOOR_T
    frame_b = 0.035

    # Door center-z so that closed leaf sits over the door region.
    if fr.fitment == "shelves_plus_drawer_stack":
        door_zc = fr.SPLIT_Z + fr.UPPER_H / 2.0
    elif fr.fitment == "shelves_plus_drawer":
        door_top = fr.INNER_H / 2.0
        door_bot = -fr.INNER_H / 2.0 + fr.LOWER_H * 0.55
        door_zc = (door_top + door_bot) / 2.0
    else:
        door_zc = 0.0

    door = model.part("cabinet_door")
    if glass:
        door.visual(mesh_from_cadquery(_build_door_frame_cq(door_w, door_h, door_t, frame_b), "door_frame"),
                    material=mats["front"], name="door_frame")
        door.visual(mesh_from_cadquery(_build_door_glass_cq(door_w, door_h, door_t, frame_b), "door_glass"),
                    material=mats["glass"], name="door_glass")
        door.visual(mesh_from_cadquery(_build_door_emblem_cq(door_w, door_t, on_glass=True), "door_emblem"),
                    material=mats["cross"], name="door_emblem")
        overlaps.append(("cabinet_door", "cabinet_door", "door_glass", "door_frame",
                         "Glazing is captured in the door frame's window rebate (seated fit)."))
        overlaps.append(("cabinet_door", "cabinet_door", "door_emblem", "door_glass",
                         "Red cross emblem is a raised decal bonded onto the glass face."))
    else:
        door.visual(mesh_from_cadquery(_build_door_panel_cq(door_w, door_h, door_t), "door_panel"),
                    material=mats["front"], name="door_frame")
        door.visual(mesh_from_cadquery(_build_door_emblem_cq(door_w, door_t, on_glass=False), "door_emblem"),
                    material=mats["cross"], name="door_emblem")
        overlaps.append(("cabinet_door", "cabinet_door", "door_emblem", "door_frame",
                         "Red cross is a printed decal bonded onto the panel face."))

    door.visual(mesh_from_cadquery(_build_door_banner_cq(door_w, door_h, door_t, frame_b), "door_banner"),
                material=mats["cross"], name="door_banner")
    door.visual(mesh_from_cadquery(_build_door_knob_cq(door_w, door_t, frame_b, 1.0), "door_knob"),
                material=mats["hardware"], name="door_knob")
    overlaps.append(("cabinet_door", "cabinet_door", "door_banner", "door_frame",
                     "FIRST AID banner is a printed decal on the upper door frame."))

    # Door-side hinge knuckles centered on the pin (door-local x=0).
    knuckle_h = min(0.040, door_h / 4.0)
    door_knuckle_z = [-door_h / 4.0, door_h / 4.0]
    door.visual(
        mesh_from_cadquery(_build_hinge_knuckles_cq(door_knuckle_z, knuckle_h), "door_hinge"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mats["hardware"],
        name="door_hinge_barrel",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=2.0,
        origin=Origin(xyz=(HINGE_R + door_w / 2.0, 0.0, 0.0)),
    )

    # Body-side hinge barrel (left front corner of the door region).
    body = model.get_part("cabinet_body")
    body_knuckle_z = [0.0]
    hinge_x = -fr.BODY_W / 2.0 + 0.006
    hinge_y = fr.BODY_D / 2.0 + HINGE_INSET
    body.visual(
        mesh_from_cadquery(_build_hinge_knuckles_cq(body_knuckle_z, knuckle_h), "body_hinge_s"),
        origin=Origin(xyz=(hinge_x, hinge_y - 0.002, door_zc + fr.Z_LIFT)),
        material=mats["hardware"],
        name="body_hinge_barrel",
    )
    overlaps.append(("cabinet_body", "cabinet_body", "body_hinge_barrel", "body_shell",
                     "Hinge barrel is embedded into the body side wall (welded mount)."))

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, door_zc + fr.Z_LIFT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=math.radians(150.0)),
    )
    # Captured-pin hinge: door barrel interleaves body barrel (grandfathered).
    overlaps.append(("cabinet_door", "cabinet_body", "door_hinge_barrel", "body_hinge_barrel",
                     "Hinge knuckles interleave around the captured pin (barrel hinge)."))
    # Closed door leaf seats lightly against the cabinet front face (shallow Y embed).
    overlaps.append(("cabinet_body", "cabinet_door", "body_shell", "door_frame",
                     "Door leaf seats against the cabinet front face at the closed position."))
    return ["body_to_door"], overlaps


def _build_double_doors(model, fr: _Frame, r, mats):
    """Two narrow leaves door_0 / door_1 from the center line, REVOLUTE +Z / -Z."""
    overlaps: list[tuple] = []
    door_h = _door_region_h(fr)
    door_t = fr.DOOR_T
    frame_b = 0.026
    # Each leaf spans half the body width, hinged on the outer edge.
    leaf_w = fr.BODY_W / 2.0 - 0.006 - HINGE_R

    if fr.fitment == "shelves_plus_drawer_stack":
        door_zc = fr.SPLIT_Z + fr.UPPER_H / 2.0
    elif fr.fitment == "shelves_plus_drawer":
        door_top = fr.INNER_H / 2.0
        door_bot = -fr.INNER_H / 2.0 + fr.LOWER_H * 0.55
        door_zc = (door_top + door_bot) / 2.0
    else:
        door_zc = 0.0

    knuckle_h = min(0.040, door_h / 4.0)
    body = model.get_part("cabinet_body")
    joints: list[str] = []

    # side: +1 = left leaf (hinge on -X, opens about +Z, free edge toward +X/center)
    #       -1 = right leaf (hinge on +X, opens about -Z, free edge toward -X/center)
    for idx, side in enumerate((1, -1)):
        name = f"door_{idx}"
        door = model.part(name)
        # Build leaf in its own local frame (hinge x=0, leaf extends +X), then
        # the joint axis sign + mirrored placement handle the symmetry.
        door.visual(mesh_from_cadquery(_build_door_frame_cq(leaf_w, door_h, door_t, frame_b), f"frame_{idx}"),
                    material=mats["front"], name="door_frame")
        door.visual(mesh_from_cadquery(_build_door_glass_cq(leaf_w, door_h, door_t, frame_b), f"glass_{idx}"),
                    material=mats["glass"], name="door_glass")
        door.visual(mesh_from_cadquery(_build_door_emblem_cq(leaf_w, door_t, on_glass=True), f"emblem_{idx}"),
                    material=mats["cross"], name="door_emblem")
        door.visual(mesh_from_cadquery(_build_door_banner_cq(leaf_w, door_h, door_t, frame_b), f"banner_{idx}"),
                    material=mats["cross"], name="door_banner")
        door.visual(mesh_from_cadquery(_build_door_knob_cq(leaf_w, door_t, frame_b, 1.0), f"knob_{idx}"),
                    material=mats["hardware"], name="door_knob")
        door_knuckle_z = [-door_h / 4.0, door_h / 4.0]
        door.visual(
            mesh_from_cadquery(_build_hinge_knuckles_cq(door_knuckle_z, knuckle_h), f"door_hinge_{idx}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=mats["hardware"],
            name="door_hinge_barrel",
        )
        door.inertial = Inertial.from_geometry(
            Box((leaf_w, door_t, door_h)),
            mass=1.4,
            origin=Origin(xyz=(HINGE_R + leaf_w / 2.0, 0.0, 0.0)),
        )

        overlaps.append((name, name, "door_glass", "door_frame",
                         "Glazing is captured in the leaf window rebate (seated fit)."))
        overlaps.append((name, name, "door_emblem", "door_glass",
                         "Red cross emblem is a raised decal bonded onto the glass."))
        overlaps.append((name, name, "door_banner", "door_frame",
                         "FIRST AID banner is a printed decal on the leaf frame."))

        # Hinge x at the outer edge of this leaf's side.
        hinge_x = side * (fr.BODY_W / 2.0 - 0.006)
        hinge_y = fr.BODY_D / 2.0 + HINGE_INSET
        # Body-side barrel for this corner.
        body.visual(
            mesh_from_cadquery(_build_hinge_knuckles_cq([0.0], knuckle_h), f"body_hinge_{idx}"),
            origin=Origin(xyz=(hinge_x, hinge_y - 0.002, door_zc + fr.Z_LIFT)),
            material=mats["hardware"],
            name=f"body_hinge_barrel_{idx}",
        )
        overlaps.append(("cabinet_body", "cabinet_body", f"body_hinge_barrel_{idx}", "body_shell",
                         "Hinge barrel embedded into the body side wall (welded mount)."))

        axis_z = 1.0 if side > 0 else -1.0
        model.articulation(
            f"body_to_door_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, hinge_y, door_zc + fr.Z_LIFT)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=math.radians(150.0)),
        )
        overlaps.append((name, "cabinet_body", "door_hinge_barrel", f"body_hinge_barrel_{idx}",
                         "Hinge knuckles interleave around the captured pin (barrel hinge)."))
        # Closed leaf seats against the cabinet front face (shallow Y embed).
        overlaps.append(("cabinet_body", name, "body_shell", "door_frame",
                         "Door leaf seats against the cabinet front face at the closed position."))
        joints.append(f"body_to_door_{idx}")
    # Two leaves meet along the centerline at closed pose (mullion seam).
    overlaps.append(("door_0", "door_1", "door_frame", "door_frame",
                     "The two leaves meet along the center mullion when closed."))
    return joints, overlaps


# ---------------------------------------------------------------------------
# Fitment / drawer modules — return (joint_names, overlaps).
# ---------------------------------------------------------------------------
def _build_drawers(model, fr: _Frame, r, mats):
    """Emit drawer_{i} (+Y PRISMATIC) for shelves_plus_drawer(_stack)."""
    overlaps: list[tuple] = []
    joints: list[str] = []
    n = r.drawer_count
    if n <= 0:
        return joints, overlaps
    body = model.get_part("cabinet_body")
    single = (r.fitment_choice == "shelves_plus_drawer")

    def _add_slide_rail(idx_tag: str, slot_z: float) -> None:
        # Thin front slide rail at this drawer's z on the body front plane, full
        # inner width so it bridges to both side walls (no island) and the joint
        # origin lands on real carcass geometry (fence_cascade / cabinet pattern).
        body.visual(
            mesh_from_cadquery(
                cq.Workplane("XY").box(fr.INNER_W, fr.WALL_T, 0.010),
                f"slide_rail_{idx_tag}",
            ),
            origin=Origin(xyz=(0.0, fr.BODY_D / 2.0 - fr.WALL_T / 2.0, slot_z + fr.Z_LIFT)),
            material=mats["body"],
            name=f"slide_rail_{idx_tag}",
        )
        overlaps.append(("cabinet_body", "cabinet_body", f"slide_rail_{idx_tag}", "body_shell",
                         "Drawer slide rail is welded onto the body front frame at the opening."))

    if single:
        slot_z = -fr.INNER_H / 2.0 + (fr.LOWER_H * 0.55) / 2.0
        _add_slide_rail("d", slot_z)
        drawer_w = min(0.260 * (fr.BODY_W / 0.340), fr.INNER_W - 0.012)
        open_w = drawer_w - 0.004
        box_h = fr.LOWER_H * 0.55 - 0.016
        tray_mesh = mesh_from_cadquery(
            _build_drawer_tray_cq(fr, drawer_w, box_h, open_w), "drawer_tray"
        )
        drw = model.part("drawer")
        drw.visual(tray_mesh, material=mats["front"], name="drawer_front")
        drw.inertial = Inertial.from_geometry(
            Box((drawer_w, fr.INNER_D, box_h)),
            mass=1.0,
            origin=Origin(xyz=(0.0, -fr.INNER_D / 2.0, 0.0)),
        )
        model.articulation(
            "body_to_drawer",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drw,
            origin=Origin(xyz=(0.0, fr.BODY_D / 2.0, slot_z + fr.Z_LIFT)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.3, lower=0.0, upper=r.drawer_travel),
        )
        overlaps.append(("cabinet_body", "drawer", None, None,
                         "Drawer tray slides inside the cavity through its front-panel opening."))
        joints.append("body_to_drawer")
        return joints, overlaps

    # Stack: n drawers, each its own +Y prismatic (uniform policy).
    slot_h = fr.LOWER_H / n
    box_h = slot_h - 0.010
    drawer_w = fr.INNER_W - 0.008
    open_w = fr.INNER_W - 0.010
    tray_mesh = mesh_from_cadquery(
        _build_drawer_tray_cq(fr, drawer_w, box_h, open_w), "drawer_tray"
    )
    for i in range(n):
        slot_z = _drawer_slot_z(fr, i, n)
        _add_slide_rail(str(i), slot_z)
        drw = model.part(f"drawer_{i}")
        drw.visual(tray_mesh, material=mats["front"], name="drawer_front")
        drw.inertial = Inertial.from_geometry(
            Box((drawer_w, fr.INNER_D, box_h)),
            mass=0.9,
            origin=Origin(xyz=(0.0, -fr.INNER_D / 2.0, 0.0)),
        )
        model.articulation(
            f"body_to_drawer_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drw,
            origin=Origin(xyz=(0.0, fr.BODY_D / 2.0, slot_z + fr.Z_LIFT)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.3, lower=0.0, upper=r.drawer_travel),
        )
        overlaps.append(("cabinet_body", f"drawer_{i}", None, None,
                         "Drawer tray slides inside the cavity through its front-panel opening."))
        joints.append(f"body_to_drawer_{i}")
    return joints, overlaps


# ---------------------------------------------------------------------------
# Top-level builder
# ---------------------------------------------------------------------------
def build_first_aid_cabinet(
    config: FirstAidCabinetConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    cfg = config or FirstAidCabinetConfig()
    r = resolve_config(cfg)
    if assets is None:
        assets = AssetContext(Path(tempfile.mkdtemp(prefix="articraft-first-aid-cabinet-")))
    model = ArticulatedObject(name=r.name, assets=assets)

    palette = PALETTES[r.palette_style]
    mats = {
        key: model.material(f"fac_{key}_{r.palette_style}", rgba=rgba)
        for key, rgba in palette.items()
    }

    fr = _make_frame(r)

    overlaps: list[tuple] = []
    overlaps += _build_carcass(model, fr, r, mats)

    # Slot A: door
    if r.door_choice == "double_doors":
        door_joints, door_overlaps = _build_double_doors(model, fr, r, mats)
    else:
        glass = r.door_choice == "glass_front_hinged"
        door_joints, door_overlaps = _build_single_door(model, fr, r, mats, glass=glass)
    overlaps += door_overlaps

    # Slot B: fitment drawers (shelves are part of the carcass).
    drawer_joints, drawer_overlaps = _build_drawers(model, fr, r, mats)
    overlaps += drawer_overlaps

    model.meta["slot_choices"] = _slot_choices_for_resolved(r)
    model.meta["_fac_overlaps"] = overlaps
    model.meta["_fac_door_joints"] = door_joints
    model.meta["_fac_drawer_joints"] = drawer_joints
    return model


def build_seeded_first_aid_cabinet(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_first_aid_cabinet(config_from_seed(seed), assets=assets)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_first_aid_cabinet_tests(
    object_model: ArticulatedObject, config: FirstAidCabinetConfig
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    part_names = {p.name for p in object_model.parts}

    ctx.check("cabinet_body present", "cabinet_body" in part_names)

    # Declare element-scoped overlaps recorded during build.
    overlaps = object_model.meta.get("_fac_overlaps", [])
    for pa, pb, ea, eb, reason in overlaps:
        if pa not in part_names or pb not in part_names:
            continue
        if ea is None:
            ctx.allow_overlap(object_model.get_part(pa), object_model.get_part(pb), reason=reason)
        else:
            ctx.allow_overlap(
                object_model.get_part(pa), object_model.get_part(pb),
                elem_a=ea, elem_b=eb, reason=reason,
            )

    # Grounding: lowest geometry sits on the floor.
    zmins = []
    for p in object_model.parts:
        ab = ctx.part_world_aabb(p)
        if ab is not None:
            zmins.append(ab[0][2])
    if zmins:
        ctx.check("cabinet rests on the floor", abs(min(zmins)) <= 0.012,
                  details=f"zmin={min(zmins):.4f}")

    # --- Door mechanism: >= 1 REVOLUTE vertical-Z door ---
    door_joints = object_model.meta.get("_fac_door_joints", [])
    ctx.check("has at least one door joint", len(door_joints) >= 1)
    for jn in door_joints:
        j = object_model.get_articulation(jn)
        ctx.check(
            f"{jn} revolute vertical Z",
            j.articulation_type == ArticulationType.REVOLUTE
            and abs(j.axis[0]) < 1e-9 and abs(j.axis[1]) < 1e-9
            and abs(abs(j.axis[2]) - 1.0) < 1e-9,
            details=str(j.axis),
        )
        lim = j.motion_limits
        ctx.check(
            f"{jn} opens (positive upper)",
            lim is not None and lim.lower == 0.0 and lim.upper is not None and lim.upper > 1.0,
            details=f"limits=({lim.lower if lim else None},{lim.upper if lim else None})",
        )

    if r.door_choice == "double_doors":
        ctx.check("two leaves present",
                  {"door_0", "door_1"} <= part_names, details=str(sorted(part_names)))
        j0 = object_model.get_articulation("body_to_door_0")
        j1 = object_model.get_articulation("body_to_door_1")
        ctx.check("double doors open opposite (axis sign mirrors)",
                  j0.axis[2] * j1.axis[2] < 0.0,
                  details=f"axis0={j0.axis}, axis1={j1.axis}")
    else:
        ctx.check("single cabinet_door present", "cabinet_door" in part_names)

    # --- Door closed pose: covers the cabinet front, in front of body face ---
    main_door = "cabinet_door" if r.door_choice != "double_doors" else "door_0"
    hinge = object_model.get_articulation(door_joints[0])
    door_part = object_model.get_part(main_door)
    body_part = object_model.get_part("cabinet_body")
    with ctx.pose({hinge: 0.0}):
        da = ctx.part_world_aabb(door_part)
        ba = ctx.part_world_aabb(body_part)
        ctx.check(
            "closed door is in front of body face",
            da is not None and ba is not None and da[1][1] > ba[1][1] - 0.006,
            details=f"door_max_y={da[1][1] if da else None}, body_max_y={ba[1][1] if ba else None}",
        )
        closed_knob = ctx.part_element_world_aabb(door_part, elem="door_knob")

    # --- Door open pose: free edge swings away ---
    with ctx.pose({hinge: math.radians(120.0)}):
        open_knob = ctx.part_element_world_aabb(door_part, elem="door_knob")
    if closed_knob is not None and open_knob is not None:
        rest_c = ((closed_knob[0][0] + closed_knob[1][0]) / 2.0,
                  (closed_knob[0][1] + closed_knob[1][1]) / 2.0)
        open_c = ((open_knob[0][0] + open_knob[1][0]) / 2.0,
                  (open_knob[0][1] + open_knob[1][1]) / 2.0)
        moved = math.hypot(open_c[0] - rest_c[0], open_c[1] - rest_c[1])
        ctx.check("opening the hinge swings the door free edge", moved > 0.06,
                  details=f"free-edge travel={moved:.3f} m")

    # --- Shelf count ---
    shelf_elems = [v.name for v in body_part.visuals if v.name and v.name.startswith("shelf_")]
    ctx.check("shelf count matches", len(shelf_elems) == r.shelf_count,
              details=f"expected {r.shelf_count}, got {len(shelf_elems)}")

    # --- Drawer mechanism (uniform +Y PRISMATIC) ---
    drawer_joints = object_model.meta.get("_fac_drawer_joints", [])
    ctx.check("drawer joint count matches drawer_count",
              len(drawer_joints) == r.drawer_count,
              details=f"expected {r.drawer_count}, got {len(drawer_joints)}")
    for jn in drawer_joints:
        j = object_model.get_articulation(jn)
        ctx.check(
            f"{jn} prismatic +Y",
            j.articulation_type == ArticulationType.PRISMATIC
            and abs(j.axis[0]) < 1e-9 and abs(j.axis[1] - 1.0) < 1e-6 and abs(j.axis[2]) < 1e-9,
            details=str(j.axis),
        )

    # --- Drawers slide forward and retract inside ---
    for jn in drawer_joints:
        j = object_model.get_articulation(jn)
        child = object_model.get_part(j.child)
        with ctx.pose({j: 0.0}):
            closed_pos = ctx.part_world_position(child)
        with ctx.pose({j: r.drawer_travel}):
            ext_pos = ctx.part_world_position(child)
        if closed_pos is not None and ext_pos is not None:
            ctx.check(f"{jn} extends forward (+Y)", ext_pos[1] - closed_pos[1] > 0.04,
                      details=f"delta_y={ext_pos[1] - closed_pos[1]:.3f}")

    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    return ctx.report()


__all__ = [
    "FirstAidCabinetConfig",
    "ResolvedFirstAidCabinetConfig",
    "build_first_aid_cabinet",
    "build_seeded_first_aid_cabinet",
    "config_from_seed",
    "resolve_config",
    "run_first_aid_cabinet_tests",
    "slot_choices_for_seed",
]
