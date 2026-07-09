from __future__ import annotations

"""Modular template: military / field two-way radio (rugged handheld walkie-talkie).

Identity = **fold-over antenna pivot + rotary knob(s) / PTT mechanism** on a
rugged upright handheld body. Derived from the parent record
``rec_model-a-rugged-military-style-handheld-two-way-r_..._3ffa3864`` and eight
sibling variants (antenna 2-seg / stub, flip-cover, rotary-selector, keypad4x3,
single-bar-PTT, knob N=2 / N=3).

Slots (parallel children — every module parents directly to the root ``body``):

* Slot A ``antenna``        — ``foldover_whip`` / ``articulated_2seg`` / ``stub``.
                              ``antenna_fold`` REVOLUTE is always present; the
                              2-seg module adds a second ``elbow_fold`` REVOLUTE.
* Slot B ``control_layout`` — ``keypad4x4_dualptt`` / ``keypad4x3`` /
                              ``flip_cover`` / ``rotary_selector`` /
                              ``single_bar_ptt``. Drives the front panel keypad
                              grid (4x4 vs 4x3), an optional flip-up cover
                              (REVOLUTE), an optional front rotary channel dial
                              (CONTINUOUS, replaces the lower keypad), and the
                              PTT type (two prismatic buttons vs one prismatic
                              bar).
* Multiplicity ``knob_count`` (1/2/3) — the single hand-written parent knob is
  loop-rewritten into ``top_knob_{i}``, each its own CONTINUOUS joint about +Z,
  evenly spaced along the top deck. Each knob carries an off-axis pointer tab so
  the axisymmetric ``KnobGeometry`` still fails an AABB no-rotation spin check.

The belt clip (``clip_hinge`` REVOLUTE) is always present.

Frame conventions: X = width (left side at -X), Y = depth (front at -Y, back at
+Y), Z = up. Body grounded at z=0.
"""

import math
import random
from dataclasses import dataclass, replace
from math import pi
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
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

__modular__ = True

# ---------------------------------------------------------------------------
# Enum domains
# ---------------------------------------------------------------------------
Antenna = Literal["foldover_whip", "articulated_2seg", "stub"]
ControlLayout = Literal[
    "keypad4x4_dualptt",
    "keypad4x3",
    "flip_cover",
    "rotary_selector",
    "single_bar_ptt",
]
PaletteStyle = Literal["od_green", "black", "tan", "coyote"]

ANTENNAS: tuple[Antenna, ...] = ("foldover_whip", "articulated_2seg", "stub")
CONTROL_LAYOUTS: tuple[ControlLayout, ...] = (
    "keypad4x4_dualptt",
    "keypad4x3",
    "flip_cover",
    "rotary_selector",
    "single_bar_ptt",
)
PALETTE_STYLES: tuple[PaletteStyle, ...] = ("od_green", "black", "tan", "coyote")

KNOB_COUNT_MIN, KNOB_COUNT_MAX = 1, 3
KEYPAD_COLS_DOMAIN: tuple[int, ...] = (3, 4)

# ---------------------------------------------------------------------------
# Per-seed palette table (>= 3 named materials each; mapped onto every visual)
# ---------------------------------------------------------------------------
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "od_green": {
        "shell_upper": (0.20, 0.26, 0.16, 1.0),
        "shell_lower": (0.30, 0.36, 0.22, 1.0),
        "graphite": (0.13, 0.14, 0.13, 1.0),
        "rubber": (0.10, 0.11, 0.10, 1.0),
        "keypad": (0.42, 0.48, 0.34, 1.0),
        "panel": (0.24, 0.29, 0.19, 1.0),
        "lcd": (0.55, 0.74, 0.30, 1.0),
        "led_red": (0.85, 0.10, 0.08, 1.0),
        "led_green": (0.15, 0.80, 0.20, 1.0),
        "clip": (0.16, 0.18, 0.15, 1.0),
        "accent": (0.70, 0.66, 0.30, 1.0),
    },
    "black": {
        "shell_upper": (0.09, 0.09, 0.10, 1.0),
        "shell_lower": (0.16, 0.16, 0.18, 1.0),
        "graphite": (0.14, 0.14, 0.15, 1.0),
        "rubber": (0.11, 0.11, 0.12, 1.0),
        "keypad": (0.30, 0.31, 0.33, 1.0),
        "panel": (0.18, 0.18, 0.20, 1.0),
        "lcd": (0.40, 0.72, 0.55, 1.0),
        "led_red": (0.88, 0.12, 0.10, 1.0),
        "led_green": (0.18, 0.82, 0.24, 1.0),
        "clip": (0.20, 0.21, 0.23, 1.0),
        "accent": (0.78, 0.79, 0.81, 1.0),
    },
    "tan": {
        "shell_upper": (0.45, 0.40, 0.28, 1.0),
        "shell_lower": (0.74, 0.63, 0.43, 1.0),
        "graphite": (0.22, 0.20, 0.16, 1.0),
        "rubber": (0.16, 0.15, 0.13, 1.0),
        "keypad": (0.83, 0.75, 0.55, 1.0),
        "panel": (0.68, 0.57, 0.38, 1.0),
        "lcd": (0.62, 0.78, 0.25, 1.0),
        "led_red": (0.85, 0.10, 0.08, 1.0),
        "led_green": (0.15, 0.80, 0.20, 1.0),
        "clip": (0.30, 0.27, 0.22, 1.0),
        "accent": (0.50, 0.34, 0.18, 1.0),
    },
    "coyote": {
        "shell_upper": (0.36, 0.30, 0.22, 1.0),
        "shell_lower": (0.58, 0.49, 0.36, 1.0),
        "graphite": (0.18, 0.16, 0.13, 1.0),
        "rubber": (0.13, 0.12, 0.11, 1.0),
        "keypad": (0.66, 0.57, 0.42, 1.0),
        "panel": (0.50, 0.42, 0.30, 1.0),
        "lcd": (0.58, 0.76, 0.34, 1.0),
        "led_red": (0.86, 0.11, 0.09, 1.0),
        "led_green": (0.16, 0.80, 0.22, 1.0),
        "clip": (0.26, 0.23, 0.19, 1.0),
        "accent": (0.78, 0.62, 0.30, 1.0),
    },
}

# ---------------------------------------------------------------------------
# Base real-world dimensions (meters). body_scale scales the X/Y/Z footprint.
# ---------------------------------------------------------------------------
BODY_W = 0.065  # X
BODY_D = 0.038  # Y
BODY_H = 0.220  # Z
SPLIT_Z = 0.128  # lower / upper color split

# top-face hardware
KNOB_DIA = 0.018
KNOB_H = 0.015
KNOB_BASE_DZ = 0.0045  # knob skirt bottom above BODY_H (boss top region)

# antenna (in body coords)
ANT_X = -0.018
ANT_PIVOT_DZ = 0.009  # antenna fold pivot above BODY_H (ANT_PIVOT_Z = BODY_H+dz)
ANT_COLLAR_H = 0.036
ANT_WHIP_LEN = 0.350
# articulated_2seg
ANT2_COLLAR_H = 0.036
ANT2_BASE_LEN = 0.080
ANT2_WHIP_LEN = 0.280
# stub
ANTS_COLLAR_H = 0.030
ANTS_STUB_LEN = 0.065

# LCD
LCD_W = 0.046
LCD_H = 0.042

# keypad
KEY_ROWS_Z = (0.0535, 0.0685, 0.0835, 0.0985)
KEY_COLS_4 = (-0.018, -0.006, 0.006, 0.018)
KEY_COLS_3 = (-0.014, 0.0, 0.014)

# PTT
PTT_Z = (0.165, 0.142)
PTT_BAR_Z = 0.1535
PTT_TRAVEL = 0.003

# belt clip
CLIP_PIVOT_DY = 0.0035  # clip pivot Y above half-depth (back boss outer face)
CLIP_PIVOT_Z = 0.168
CLIP_OPEN = 25.0 * pi / 180.0

# flip cover
COVER_WIDTH = 0.056
COVER_HEIGHT = 0.162
COVER_THICK = 0.002
COVER_HINGE_Z = 0.212
COVER_OPEN = 160.0 * pi / 180.0

# rotary selector dial (front face, replaces lower keypad)
DIAL_CZ = 0.076
DIAL_DIA = 0.048
DIAL_R = DIAL_DIA / 2.0
DIAL_H = 0.012


# ---------------------------------------------------------------------------
# Config dataclasses
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class FieldRadioConfig:
    antenna: Antenna | None = None
    control_layout: ControlLayout | None = None
    palette_style: PaletteStyle = "od_green"
    knob_count: int | None = None
    keypad_cols: int | None = None
    body_scale: float = 1.0
    name: str = "field_radio"


@dataclass(frozen=True)
class ResolvedFieldRadioConfig:
    antenna: Antenna
    control_layout: ControlLayout
    palette_style: PaletteStyle
    knob_count: int
    keypad_cols: int
    body_scale: float
    # derived semantic flags
    has_keypad: bool
    has_flip_cover: bool
    has_rotary_dial: bool
    ptt_single_bar: bool
    name: str

    @property
    def s(self) -> float:
        return self.body_scale


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _pick(value, choices):
    return value if value in choices else choices[0]


# ---------------------------------------------------------------------------
# Seed sampling + resolution
# ---------------------------------------------------------------------------
def config_from_seed(seed: int) -> FieldRadioConfig:
    rng = random.Random(seed)
    # Procedural-first: sample structural slots, then the knob multiplicity,
    # then palette, then a continuous body scale. seed 0 is not special.
    antenna = rng.choices(ANTENNAS, weights=[5, 3, 3], k=1)[0]
    control = rng.choices(CONTROL_LAYOUTS, weights=[5, 4, 3, 3, 3], k=1)[0]
    # knob_count weighted toward the parent N=1 but the gate wants N=2/3 sampled.
    knob_count = rng.choices([1, 2, 3], weights=[4, 4, 3], k=1)[0]
    keypad_cols = rng.choice(KEYPAD_COLS_DOMAIN)
    return FieldRadioConfig(
        antenna=antenna,
        control_layout=control,
        palette_style=rng.choice(PALETTE_STYLES),
        knob_count=knob_count,
        keypad_cols=keypad_cols,
        body_scale=round(rng.uniform(0.9, 1.15), 4),
        name=f"seeded_field_radio_{seed}",
    )


def resolve_config(
    config: FieldRadioConfig | None = None,
) -> ResolvedFieldRadioConfig:
    cfg = config or FieldRadioConfig()
    antenna = _pick(cfg.antenna, ANTENNAS)
    control = _pick(cfg.control_layout, CONTROL_LAYOUTS)
    palette_style = _pick(cfg.palette_style, PALETTE_STYLES)

    knob_count = int(
        _clamp(cfg.knob_count if cfg.knob_count is not None else 1, KNOB_COUNT_MIN, KNOB_COUNT_MAX)
    )
    keypad_cols = cfg.keypad_cols if cfg.keypad_cols in KEYPAD_COLS_DOMAIN else 4
    # keypad4x3 forces a 3-column grid (the secondary multiplicity axis).
    if control == "keypad4x3":
        keypad_cols = 3

    has_rotary_dial = control == "rotary_selector"
    has_flip_cover = control == "flip_cover"
    # rotary_selector replaces the lower keypad with the dial; all other
    # layouts keep the front keypad grid.
    has_keypad = not has_rotary_dial
    ptt_single_bar = control == "single_bar_ptt"

    body_scale = _clamp(cfg.body_scale, 0.9, 1.15)

    return ResolvedFieldRadioConfig(
        antenna=antenna,
        control_layout=control,
        palette_style=palette_style,
        knob_count=knob_count,
        keypad_cols=keypad_cols,
        body_scale=body_scale,
        has_keypad=has_keypad,
        has_flip_cover=has_flip_cover,
        has_rotary_dial=has_rotary_dial,
        ptt_single_bar=ptt_single_bar,
        name=cfg.name or "field_radio",
    )


def with_overrides(config: FieldRadioConfig, **kwargs: object) -> FieldRadioConfig:
    return replace(config, **kwargs)


def slot_choices_for_config(
    config: FieldRadioConfig | ResolvedFieldRadioConfig,
) -> tuple[tuple[str, str], ...]:
    r = config if isinstance(config, ResolvedFieldRadioConfig) else resolve_config(config)
    choices = [
        ("antenna", r.antenna),
        ("control_layout", r.control_layout),
        ("knob_count", f"knob_{r.knob_count}"),
    ]
    # keypad grid is a secondary multiplicity axis only when a keypad exists.
    if r.has_keypad:
        choices.append(("keypad", f"cols_{r.keypad_cols}"))
    return tuple(choices)


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


# ---------------------------------------------------------------------------
# CadQuery shape helpers (adapted verbatim from the 5-star sources, with the
# few literal dimensions promoted to scale-aware values).
# ---------------------------------------------------------------------------
def _rounded_slab(w: float, d: float, z0: float, z1: float, rad: float) -> cq.Workplane:
    slab = (
        cq.Workplane("XY")
        .box(w, d, z1 - z0, centered=(True, True, False))
        .edges("|Z")
        .fillet(rad)
    )
    return slab.translate((0.0, 0.0, z0))


def _lower_shell_shape(s: float) -> cq.Workplane:
    return _rounded_slab(BODY_W * s, BODY_D * s, 0.0, SPLIT_Z * s + 0.001, 0.006)


def _upper_shell_shape(s: float) -> cq.Workplane:
    shell = _rounded_slab(BODY_W * s, BODY_D * s, SPLIT_Z * s, BODY_H * s, 0.006)
    try:
        shell = shell.edges(">Z").fillet(0.0025)
    except Exception:
        pass
    return shell


def _lcd_bezel_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(LCD_W, 0.004, LCD_H)
    window = cq.Workplane("XY").box(0.034, 0.012, 0.030)
    return frame.cut(window)


def _ptt_button_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(0.007, 0.024, 0.013)
        .edges("|X")
        .fillet(0.0045)
        .translate((-0.0025, 0.0, 0.0))
    )
    stem = cq.Workplane("XY").box(0.008, 0.014, 0.008).translate((0.003, 0.0, 0.0))
    return cap.union(stem)


def _ptt_bar_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(0.008, 0.024, 0.055)
        .edges("|X")
        .fillet(0.0045)
        .translate((-0.003, 0.0, 0.0))
    )
    stem = cq.Workplane("XY").box(0.008, 0.014, 0.040).translate((0.003, 0.0, 0.0))
    return cap.union(stem)


def _belt_clip_shape() -> cq.Workplane:
    blade = (
        cq.Workplane("XY")
        .box(0.020, 0.0025, 0.108)
        .translate((0.0, 0.00175, -0.043))
    )
    tip = (
        cq.Workplane("XY")
        .box(0.020, 0.0025, 0.016)
        .rotate((0, 0, 0), (1, 0, 0), 20.0)
        .translate((0.0, 0.0044, -0.1015))
    )
    screw = (
        cq.Workplane("XY")
        .cylinder(0.0035, 0.004, centered=(True, True, False))
        .rotate((0, 0, 0), (1, 0, 0), -90.0)
        .translate((0.0, -0.0008, 0.0))
    )
    return blade.union(tip).union(screw)


# ----- antenna: foldover_whip (parent) -----
def _antenna_whip_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").cylinder(
        ANT_COLLAR_H + 0.001, 0.0065, centered=(True, True, False)
    ).translate((0.0, 0.0, -0.001))
    for rib_z in (0.004, 0.012, 0.020, 0.028):
        rib = (
            cq.Workplane("XY")
            .cylinder(0.004, 0.008, centered=(True, True, False))
            .translate((0.0, 0.0, rib_z))
        )
        collar = collar.union(rib)
    whip = cq.Workplane("XY").add(
        cq.Solid.makeCone(0.0045, 0.0016, ANT_WHIP_LEN)
    ).translate((0.0, 0.0, ANT_COLLAR_H - 0.002))
    tip = cq.Workplane("XY").sphere(0.0024).translate(
        (0.0, 0.0, ANT_COLLAR_H - 0.002 + ANT_WHIP_LEN)
    )
    return collar.union(whip).union(tip)


# ----- antenna: articulated_2seg (base + whip) -----
def _antenna_base_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").cylinder(
        ANT2_COLLAR_H + 0.001, 0.0065, centered=(True, True, False)
    ).translate((0.0, 0.0, -0.001))
    for rib_z in (0.004, 0.012, 0.020, 0.028):
        rib = (
            cq.Workplane("XY")
            .cylinder(0.004, 0.008, centered=(True, True, False))
            .translate((0.0, 0.0, rib_z))
        )
        collar = collar.union(rib)
    stub_h = ANT2_BASE_LEN - ANT2_COLLAR_H + 0.008
    stub = cq.Workplane("XY").cylinder(
        stub_h, 0.0045, centered=(True, True, False)
    ).translate((0.0, 0.0, ANT2_COLLAR_H))
    pivot_h = 0.016
    pivot = (
        cq.Workplane("XY")
        .cylinder(pivot_h, 0.007, centered=(True, True, False))
        .translate((0.0, 0.0, ANT2_BASE_LEN - pivot_h / 2.0))
    )
    return collar.union(stub).union(pivot)


def _antenna_seg_whip_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").cylinder(
        0.014, 0.0075, centered=(True, True, False)
    ).translate((0.0, 0.0, -0.007))
    whip_start_z = 0.004
    whip = cq.Workplane("XY").add(
        cq.Solid.makeCone(0.0042, 0.0015, ANT2_WHIP_LEN)
    ).translate((0.0, 0.0, whip_start_z))
    tip = cq.Workplane("XY").sphere(0.0022).translate(
        (0.0, 0.0, whip_start_z + ANT2_WHIP_LEN)
    )
    return collar.union(whip).union(tip)


# ----- antenna: stub (rubber duck) -----
def _antenna_stub_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").cylinder(
        ANTS_COLLAR_H + 0.001, 0.007, centered=(True, True, False)
    ).translate((0.0, 0.0, -0.001))
    for rib_z in (0.004, 0.011, 0.018, 0.025):
        rib = (
            cq.Workplane("XY")
            .cylinder(0.0035, 0.009, centered=(True, True, False))
            .translate((0.0, 0.0, rib_z))
        )
        collar = collar.union(rib)
    stub = cq.Workplane("XY").add(
        cq.Solid.makeCone(0.008, 0.0055, ANTS_STUB_LEN)
    ).translate((0.0, 0.0, ANTS_COLLAR_H))
    for i in range(5):
        groove_z = ANTS_COLLAR_H + 0.008 + i * 0.012
        r = 0.008 - (groove_z - ANTS_COLLAR_H) / ANTS_STUB_LEN * 0.0025
        groove = (
            cq.Workplane("XY")
            .cylinder(0.0015, r + 0.001, centered=(True, True, False))
            .translate((0.0, 0.0, groove_z))
        )
        stub = stub.union(groove)
    tip = cq.Workplane("XY").sphere(0.006).translate(
        (0.0, 0.0, ANTS_COLLAR_H + ANTS_STUB_LEN)
    )
    return collar.union(stub).union(tip)


# ----- flip cover -----
def _cover_panel_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(COVER_WIDTH, COVER_THICK, COVER_HEIGHT)
        .edges("|Y")
        .fillet(0.003)
        .translate((0.0, -COVER_THICK / 2.0, -COVER_HEIGHT / 2.0 - 0.002))
    )
    lip_w = COVER_WIDTH - 0.004
    lip_h = COVER_HEIGHT - 0.004
    lip = (
        cq.Workplane("XY")
        .box(lip_w, 0.001, lip_h)
        .edges("|Y")
        .fillet(0.002)
        .translate((0.0, -COVER_THICK - 0.0005, -COVER_HEIGHT / 2.0 - 0.002))
    )
    cutout = (
        cq.Workplane("XY")
        .box(lip_w - 0.005, 0.002, lip_h - 0.005)
        .edges("|Y")
        .fillet(0.001)
        .translate((0.0, -COVER_THICK - 0.0005, -COVER_HEIGHT / 2.0 - 0.002))
    )
    lip = lip.cut(cutout)
    barrel = (
        cq.Workplane("XY")
        .cylinder(0.028, 0.003, centered=(True, True, False))
        .rotate((0, 0, 0), (1, 0, 0), 90.0)
        .translate((0.0, 0.0, -0.001))
    )
    return panel.union(lip).union(barrel)


def _cover_hinge_boss_shape() -> cq.Workplane:
    boss = (
        cq.Workplane("XY")
        .box(0.030, 0.008, 0.012)
        .edges("|Y")
        .fillet(0.002)
        .translate((0.0, -0.002, -0.006))
    )
    pin = (
        cq.Workplane("XY")
        .cylinder(0.034, 0.0018, centered=(True, True, False))
        .rotate((0, 0, 0), (1, 0, 0), 90.0)
        .translate((0.0, -0.006, 0.0))
    )
    return boss.union(pin)


def _knob_diameter(n: int) -> float:
    """Knob cap diameter shrinks as the count rises so all knobs fit clear of
    each other and of the antenna on the ~65 mm top deck."""
    if n == 1:
        return KNOB_DIA  # 0.018
    if n == 2:
        return 0.015
    return 0.011


def _make_knob_geometry(n: int) -> KnobGeometry:
    """Shared knurled knob geometry (from parent / knob2 / knob3 sources).
    Cap diameter is count-aware so multi-knob layouts pack without overlap."""
    return KnobGeometry(
        _knob_diameter(n),
        KNOB_H,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=32, depth=0.0008, helix_angle_deg=18.0),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,
    )


def _knob_positions(n: int) -> tuple[float, ...]:
    """Evenly-spaced knob X positions on the top deck. All sit clear of the
    antenna boss column at ANT_X = -0.018 (the antenna collar occupies roughly
    x in [-0.026, -0.010]); spacing exceeds the count-aware cap diameter so the
    knobs never collide with each other."""
    if n == 1:
        return (0.018,)
    if n == 2:
        return (0.005, 0.024)
    return (0.001, 0.013, 0.025)


# ---------------------------------------------------------------------------
# Module emitters (all parent to the single root ``body``)
# ---------------------------------------------------------------------------
def _emit_body(model, r, mats) -> object:
    s = r.s
    body = model.part("body")
    front_y = -BODY_D * s / 2.0
    back_y = BODY_D * s / 2.0
    body_h = BODY_H * s

    body.visual(
        mesh_from_cadquery(_lower_shell_shape(s), "lower_shell", tolerance=0.0004),
        material=mats["shell_lower"],
        name="lower_shell",
    )
    body.visual(
        mesh_from_cadquery(_upper_shell_shape(s), "upper_shell", tolerance=0.0004),
        material=mats["shell_upper"],
        name="upper_shell",
    )

    # LCD bezel + recessed screen (always present, scales in Z with the body).
    lcd_cz = 0.185 * s
    body.visual(
        mesh_from_cadquery(_lcd_bezel_shape(), "lcd_bezel", tolerance=0.0003),
        origin=Origin(xyz=(0.0, front_y, lcd_cz)),
        material=mats["graphite"],
        name="lcd_bezel",
    )
    body.visual(
        Box((0.034, 0.002, 0.030)),
        origin=Origin(xyz=(0.0, front_y - 0.0005, lcd_cz)),
        material=mats["lcd"],
        name="lcd_screen",
    )

    # Indicator LEDs beside the LCD.
    body.visual(
        Cylinder(radius=0.0016, length=0.003),
        origin=Origin(xyz=(0.027, front_y - 0.0005, 0.198 * s), rpy=(pi / 2.0, 0.0, 0.0)),
        material=mats["led_red"],
        name="led_red",
    )
    body.visual(
        Cylinder(radius=0.0016, length=0.003),
        origin=Origin(xyz=(0.027, front_y - 0.0005, 0.188 * s), rpy=(pi / 2.0, 0.0, 0.0)),
        material=mats["led_green"],
        name="led_green",
    )

    # Top-face mounting bosses for the knobs (loop-driven multiplicity). Boss
    # radius tracks the count-aware knob cap so the boss really anchors it.
    boss_r = min(0.0095, _knob_diameter(r.knob_count) / 2.0 + 0.001)
    for i, knob_x in enumerate(_knob_positions(r.knob_count)):
        body.visual(
            Cylinder(radius=boss_r, length=0.005),
            origin=Origin(xyz=(knob_x, 0.0, body_h + 0.0015)),
            material=mats["graphite"],
            name=f"knob_boss_{i}",
        )

    # Antenna mounting boss.
    body.visual(
        Cylinder(radius=0.0085, length=0.010),
        origin=Origin(xyz=(ANT_X, 0.0, body_h + 0.004)),
        material=mats["graphite"],
        name="antenna_boss",
    )

    # Back boss carrying the belt-clip screw. Outer face reaches the clip
    # pivot so the screw head embeds into it (matches parent geometry).
    body.visual(
        Box((0.022, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, back_y + 0.0015, CLIP_PIVOT_Z * s)),
        material=mats["shell_upper"],
        name="clip_boss",
    )

    return body


def _emit_keypad(model, r, body, mats) -> None:
    """Front rubber keypad: tan inset panel + raised keys (4x4 or 4x3 grid)."""
    s = r.s
    front_y = -BODY_D * s / 2.0
    cols = KEY_COLS_4 if r.keypad_cols == 4 else KEY_COLS_3
    body.visual(
        Box((0.054, 0.003, 0.062)),
        origin=Origin(xyz=(0.0, front_y - 0.0005, 0.076 * s)),
        material=mats["panel"],
        name="keypad_panel",
    )
    for row, key_z in enumerate(KEY_ROWS_Z):
        for col, key_x in enumerate(cols):
            body.visual(
                Box((0.0095, 0.003, 0.0075)),
                origin=Origin(xyz=(key_x, front_y - 0.0025, key_z * s)),
                material=mats["keypad"],
                name=f"key_{row}_{col}",
            )


def _emit_rotary_dial(model, r, body, mats) -> None:
    """Large front rotary channel selector (CONTINUOUS) replacing the lower
    keypad. The dial spins about the front-face normal (+Y) and carries an
    off-axis pointer nub proving rotation."""
    s = r.s
    front_y = -BODY_D * s / 2.0
    dial_cz = DIAL_CZ * s

    # Bezel ring (decorative) inlined as a body visual.
    bezel = cq.Workplane("XZ").circle(DIAL_R + 0.004).extrude(0.003)
    bezel = bezel.cut(cq.Workplane("XZ").circle(DIAL_R + 0.001).extrude(0.004))
    body.visual(
        mesh_from_cadquery(bezel.translate((0.0, front_y, dial_cz)), "dial_bezel", tolerance=0.0003),
        material=mats["graphite"],
        name="dial_bezel",
    )

    dial = model.part("channel_dial")
    dial_geom = KnobGeometry(
        DIAL_DIA,
        DIAL_H,
        body_style="faceted",
        base_diameter=DIAL_DIA + 0.002,
        grip=KnobGrip(style="fluted", count=16, depth=0.003),
        indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(dial_geom, "channel_dial_disc"),
        origin=Origin(xyz=(0.0, DIAL_H / 2.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=mats["accent"],
        name="dial_body",
    )
    # Shaft from the dial back face into the housing (intentional embedding).
    dial.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=mats["graphite"],
        name="dial_shaft",
    )
    # Off-axis pointer nub (proves continuous rotation about the front normal).
    dial.visual(
        Box((0.004, 0.002, 0.004)),
        origin=Origin(xyz=(0.0, -DIAL_H / 2.0 - 0.001, DIAL_R - 0.006)),
        material=mats["led_red"],
        name="dial_pointer",
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, front_y, dial_cz)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )


def _emit_knobs(model, r, body, mats) -> list[str]:
    """Loop-rewrite the single parent knob into N independent CONTINUOUS
    ``top_knob_{i}`` parts. Each carries an off-axis pointer tab so the
    axisymmetric KnobGeometry fails an AABB no-rotation spin check."""
    s = r.s
    body_h = BODY_H * s
    knob_mesh = mesh_from_geometry(_make_knob_geometry(r.knob_count), "knob_cap")
    names: list[str] = []
    for i, knob_x in enumerate(_knob_positions(r.knob_count)):
        knob = model.part(f"top_knob_{i}")
        knob.visual(knob_mesh, material=mats["rubber"], name="knob_cap")
        # Hidden shaft bridging knob to the boss/body (intentional embedding).
        knob.visual(
            Cylinder(radius=0.0035, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=mats["graphite"],
            name="knob_shaft",
        )
        # Off-axis pointer tab merged onto the knob top.
        knob.visual(
            Box((0.0024, 0.0024, 0.0014)),
            origin=Origin(xyz=(0.0, -0.0062, KNOB_H + 0.0005)),
            material=mats["accent"],
            name="knob_pointer",
        )
        model.articulation(
            f"knob_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, 0.0, body_h + KNOB_BASE_DZ)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=6.0),
        )
        names.append(f"top_knob_{i}")
    return names


def _emit_antenna(model, r, body, mats) -> list[str]:
    """Slot A. antenna_fold REVOLUTE always present; 2seg adds elbow_fold."""
    s = r.s
    body_h = BODY_H * s
    ant_pivot_z = body_h + ANT_PIVOT_DZ

    if r.antenna == "articulated_2seg":
        base = model.part("antenna_base")
        base.visual(
            mesh_from_cadquery(_antenna_base_shape(), "antenna_base", tolerance=0.0002),
            material=mats["rubber"],
            name="base_segment",
        )
        model.articulation(
            "antenna_fold",
            ArticulationType.REVOLUTE,
            parent=body,
            child=base,
            origin=Origin(xyz=(ANT_X, 0.0, ant_pivot_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=pi / 2.0),
        )
        whip = model.part("antenna_whip")
        whip.visual(
            mesh_from_cadquery(_antenna_seg_whip_shape(), "antenna_whip", tolerance=0.0002),
            material=mats["rubber"],
            name="whip_segment",
        )
        model.articulation(
            "elbow_fold",
            ArticulationType.REVOLUTE,
            parent=base,
            child=whip,
            origin=Origin(xyz=(0.0, 0.0, ANT2_BASE_LEN)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=pi / 2.0),
        )
        return ["antenna_base", "antenna_whip"]

    # single-part antennas (foldover_whip / stub)
    antenna = model.part("whip_antenna")
    shape = _antenna_stub_shape() if r.antenna == "stub" else _antenna_whip_shape()
    antenna.visual(
        mesh_from_cadquery(shape, "whip_antenna", tolerance=0.0002),
        material=mats["rubber"],
        name="antenna_body",
    )
    model.articulation(
        "antenna_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=antenna,
        origin=Origin(xyz=(ANT_X, 0.0, ant_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=pi / 2.0),
    )
    return ["whip_antenna"]


def _emit_ptt(model, r, body, mats) -> list[str]:
    """Slot B PTT: two prismatic buttons, or one elongated prismatic bar."""
    s = r.s
    left_x = -BODY_W * s / 2.0
    if r.ptt_single_bar:
        ptt = model.part("ptt_bar")
        ptt.visual(
            mesh_from_cadquery(_ptt_bar_shape(), "ptt_bar", tolerance=0.0003),
            material=mats["rubber"],
            name="ptt_cap",
        )
        model.articulation(
            "ptt_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=ptt,
            origin=Origin(xyz=(left_x, 0.0, PTT_BAR_Z * s)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.1, lower=0.0, upper=PTT_TRAVEL),
        )
        return ["ptt_bar"]

    ptt_mesh = mesh_from_cadquery(_ptt_button_shape(), "ptt_button", tolerance=0.0003)
    names: list[str] = []
    for ptt_name, ptt_z in (("ptt_button_upper", PTT_Z[0]), ("ptt_button_lower", PTT_Z[1])):
        ptt = model.part(ptt_name)
        ptt.visual(ptt_mesh, material=mats["rubber"], name="ptt_cap")
        model.articulation(
            ptt_name.replace("button", "press"),
            ArticulationType.PRISMATIC,
            parent=body,
            child=ptt,
            origin=Origin(xyz=(left_x, 0.0, ptt_z * s)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.1, lower=0.0, upper=PTT_TRAVEL),
        )
        names.append(ptt_name)
    return names


def _emit_flip_cover(model, r, body, mats) -> None:
    """Flip-up protective cover over the LCD/keypad (REVOLUTE 0-160 deg)."""
    s = r.s
    front_y = -BODY_D * s / 2.0
    hinge_z = COVER_HINGE_Z * s
    hinge_y = front_y - 0.006

    # Hinge boss inlined as a body visual carrying the cover pin.
    body.visual(
        mesh_from_cadquery(_cover_hinge_boss_shape(), "cover_hinge_boss", tolerance=0.0003),
        origin=Origin(xyz=(0.0, front_y, hinge_z)),
        material=mats["graphite"],
        name="cover_hinge_boss",
    )

    cover = model.part("front_cover")
    cover.visual(
        mesh_from_cadquery(_cover_panel_shape(), "front_cover", tolerance=0.0003),
        material=mats["shell_upper"],
        name="cover_panel",
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=COVER_OPEN),
    )


def _emit_belt_clip(model, r, body, mats) -> None:
    s = r.s
    back_y = BODY_D * s / 2.0
    clip = model.part("belt_clip")
    clip.visual(
        mesh_from_cadquery(_belt_clip_shape(), "belt_clip", tolerance=0.0003),
        material=mats["clip"],
        name="clip_blade",
    )
    model.articulation(
        "clip_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=clip,
        origin=Origin(xyz=(0.0, back_y + CLIP_PIVOT_DY, CLIP_PIVOT_Z * s)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=CLIP_OPEN),
    )


# ---------------------------------------------------------------------------
# Top-level builders
# ---------------------------------------------------------------------------
def build_field_radio(
    config: FieldRadioConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=r.name, assets=assets)

    palette = PALETTES[r.palette_style]
    mats = {
        key: model.material(f"fr_{key}_{r.palette_style}", rgba=rgba)
        for key, rgba in palette.items()
    }

    body = _emit_body(model, r, mats)

    # Multiplicity axis: knobs.
    _emit_knobs(model, r, body, mats)

    # Slot A: antenna.
    _emit_antenna(model, r, body, mats)

    # Slot B: control layout — keypad / rotary dial, PTT type, optional cover.
    if r.has_keypad:
        _emit_keypad(model, r, body, mats)
    if r.has_rotary_dial:
        _emit_rotary_dial(model, r, body, mats)
    _emit_ptt(model, r, body, mats)
    if r.has_flip_cover:
        _emit_flip_cover(model, r, body, mats)

    # Always-present belt clip.
    _emit_belt_clip(model, r, body, mats)

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_field_radio(
    seed: int,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    return build_field_radio(config_from_seed(seed), assets=assets)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_field_radio_tests(
    object_model: ArticulatedObject,
    config: FieldRadioConfig,
) -> TestReport:
    r = resolve_config(config)
    s = r.s
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    left_x = -BODY_W * s / 2.0
    body_h = BODY_H * s

    # ---- intentional embeddings (element-scoped) ----
    for i in range(r.knob_count):
        knob = object_model.get_part(f"top_knob_{i}")
        ctx.allow_overlap(
            knob, body, reason="knob shaft seats through the top boss into the housing",
            elem_a="knob_shaft", elem_b=f"knob_boss_{i}",
        )
        ctx.allow_overlap(
            knob, body, reason="knob shaft passes into the upper housing shell",
            elem_a="knob_shaft", elem_b="upper_shell",
        )

    if r.antenna == "articulated_2seg":
        base = object_model.get_part("antenna_base")
        whip = object_model.get_part("antenna_whip")
        ctx.allow_overlap(
            base, body, reason="antenna base collar seats into its mounting boss",
            elem_a="base_segment", elem_b="antenna_boss",
        )
        ctx.allow_overlap(
            whip, base, reason="whip collar wraps the base elbow pivot housing",
            elem_a="whip_segment", elem_b="base_segment",
        )
    else:
        antenna = object_model.get_part("whip_antenna")
        ctx.allow_overlap(
            antenna, body, reason="antenna collar seats into its mounting boss",
            elem_a="antenna_body", elem_b="antenna_boss",
        )

    clip = object_model.get_part("belt_clip")
    ctx.allow_overlap(
        clip, body, reason="clip screw head embeds into the back screw boss",
        elem_a="clip_blade", elem_b="clip_boss",
    )

    ptt_names = (
        ["ptt_bar"] if r.ptt_single_bar else ["ptt_button_upper", "ptt_button_lower"]
    )
    for pn in ptt_names:
        ptt = object_model.get_part(pn)
        ctx.allow_overlap(
            ptt, body, reason="PTT cap rim and stem pass through the housing wall",
            elem_a="ptt_cap", elem_b="upper_shell",
        )
        ctx.allow_overlap(
            ptt, body, reason="PTT stem may cross the housing color split line",
            elem_a="ptt_cap", elem_b="lower_shell",
        )

    if r.has_rotary_dial:
        dial = object_model.get_part("channel_dial")
        ctx.allow_overlap(
            dial, body, reason="dial shaft seats through the bezel into the housing",
            elem_a="dial_shaft", elem_b="upper_shell",
        )
        ctx.allow_overlap(
            dial, body, reason="dial shaft passes into the lower housing shell",
            elem_a="dial_shaft", elem_b="lower_shell",
        )
        ctx.allow_overlap(
            dial, body, reason="dial disc seats inside the bezel ring",
            elem_a="dial_body", elem_b="dial_bezel",
        )
        ctx.allow_overlap(
            dial, body, reason="recessed dial disc seats into the front of the upper shell",
            elem_a="dial_body", elem_b="upper_shell",
        )
        ctx.allow_overlap(
            dial, body, reason="recessed dial disc seats into the front of the lower shell",
            elem_a="dial_body", elem_b="lower_shell",
        )

    if r.has_flip_cover:
        cover = object_model.get_part("front_cover")
        ctx.allow_overlap(
            cover, body, reason="cover hinge barrel wraps the body hinge pin (captured pin)",
            elem_a="cover_panel", elem_b="cover_hinge_boss",
        )

    # ---- baseline gates ----
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_joint_mating_has_gap()

    # ---- antenna fold REVOLUTE always present (identity) ----
    antenna_fold = object_model.get_articulation("antenna_fold")
    ctx.check(
        "antenna_fold_is_revolute_0_to_90deg",
        antenna_fold.articulation_type == ArticulationType.REVOLUTE
        and abs(antenna_fold.motion_limits.lower - 0.0) < 1e-9
        and abs(antenna_fold.motion_limits.upper - pi / 2.0) < 1e-6,
        details=f"type={antenna_fold.articulation_type}, limits=("
        f"{antenna_fold.motion_limits.lower}, {antenna_fold.motion_limits.upper})",
    )
    if r.antenna == "articulated_2seg":
        elbow = object_model.get_articulation("elbow_fold")
        ctx.check(
            "elbow_fold_is_revolute",
            elbow.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={elbow.articulation_type}",
        )

    # ---- knob multiplicity: N independent CONTINUOUS joints, loop-rewritten ----
    expected_n = r.knob_count
    actual_knobs = [
        p.name for p in object_model.parts if p.name.startswith("top_knob_")
    ]
    ctx.check(
        "knob_count_matches_multiplicity",
        len(actual_knobs) == expected_n,
        details=f"emitted={sorted(actual_knobs)} expected_n={expected_n}",
    )
    for i in range(expected_n):
        j = object_model.get_articulation(f"knob_spin_{i}")
        ctx.check(
            f"knob_spin_{i}_is_continuous_vertical",
            j.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(c, 3) for c in j.axis) == (0.0, 0.0, 1.0),
            details=f"type={j.articulation_type} axis={tuple(j.axis)}",
        )

    # Off-axis pointer proves each knob actually rotates (axisymmetric body
    # alone would not — the merged tab makes the AABB sweep).
    j0 = object_model.get_articulation("knob_spin_0")
    knob0 = object_model.get_part("top_knob_0")
    p0 = ctx.part_element_world_aabb(knob0, elem="knob_pointer")
    if p0 is not None:
        y0 = 0.5 * (p0[0][1] + p0[1][1])
        with ctx.pose({j0: pi}):
            p1 = ctx.part_element_world_aabb(knob0, elem="knob_pointer")
        y1 = 0.5 * (p1[0][1] + p1[1][1])
        ctx.check(
            "knob_pointer_sweeps_off_axis",
            abs(y1 - y0) > 0.010,
            details=f"pointer y q=0:{y0:.5f} q=pi:{y1:.5f}",
        )

    # ---- rotary dial: CONTINUOUS about the front normal, pointer sweeps ----
    if r.has_rotary_dial:
        jd = object_model.get_articulation("dial_spin")
        ctx.check(
            "dial_spin_is_continuous_about_y",
            jd.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(c, 3) for c in jd.axis) == (0.0, 1.0, 0.0),
            details=f"type={jd.articulation_type} axis={tuple(jd.axis)}",
        )
        dial = object_model.get_part("channel_dial")
        q0 = ctx.part_element_world_aabb(dial, elem="dial_pointer")
        if q0 is not None:
            z0 = 0.5 * (q0[0][2] + q0[1][2])
            with ctx.pose({jd: pi}):
                q1 = ctx.part_element_world_aabb(dial, elem="dial_pointer")
            z1 = 0.5 * (q1[0][2] + q1[1][2])
            ctx.check(
                "dial_pointer_sweeps_off_axis",
                abs(z1 - z0) > 0.010,
                details=f"pointer z q=0:{z0:.5f} q=pi:{z1:.5f}",
            )

    # ---- keypad: correct grid, keys proud of panel ----
    body_visual_names = {v.name for v in body.visuals}
    if r.has_keypad:
        cols = 4 if r.keypad_cols == 4 else 3
        missing = [
            f"key_{rr}_{cc}"
            for rr in range(4)
            for cc in range(cols)
            if f"key_{rr}_{cc}" not in body_visual_names
        ]
        ctx.check(
            "keypad_has_full_grid",
            not missing,
            details=f"cols={cols} missing={missing}",
        )
        key_aabb = ctx.part_element_world_aabb(body, elem="key_0_0")
        panel_aabb = ctx.part_element_world_aabb(body, elem="keypad_panel")
        if key_aabb is not None and panel_aabb is not None:
            ctx.check(
                "keys_proud_of_keypad_panel",
                key_aabb[0][1] < panel_aabb[0][1],
                details=f"key front y={key_aabb[0][1]:.5f} panel front y={panel_aabb[0][1]:.5f}",
            )
    else:
        ctx.check(
            "rotary_layout_has_no_keypad_panel",
            "keypad_panel" not in body_visual_names,
            details="rotary_selector replaces the keypad with the dial",
        )

    # ---- PTT: prismatic press inward ----
    for pn in ptt_names:
        jn = "ptt_press" if r.ptt_single_bar else pn.replace("button", "press")
        j = object_model.get_articulation(jn)
        ctx.check(
            f"{jn}_prismatic_3mm_inward",
            j.articulation_type == ArticulationType.PRISMATIC
            and abs(j.motion_limits.upper - PTT_TRAVEL) < 1e-9
            and tuple(round(c, 3) for c in j.axis) == (1.0, 0.0, 0.0),
            details=f"axis={tuple(j.axis)} upper={j.motion_limits.upper}",
        )
    # A PTT presses inward by its stroke.
    ptt0 = object_model.get_part(ptt_names[0])
    jp = "ptt_press" if r.ptt_single_bar else ptt_names[0].replace("button", "press")
    jpress = object_model.get_articulation(jp)
    cap0 = ctx.part_world_aabb(ptt0)
    ctx.check(
        "ptt_proud_of_left_face",
        cap0[0][0] < left_x - 0.003,
        details=f"ptt xmin={cap0[0][0]:.5f} left face x={left_x:.5f}",
    )
    with ctx.pose({jpress: PTT_TRAVEL}):
        cap1 = ctx.part_world_aabb(ptt0)
    ctx.check(
        "ptt_presses_inward_3mm",
        abs((cap1[0][0] - cap0[0][0]) - PTT_TRAVEL) < 1e-5,
        details=f"rest xmin={cap0[0][0]:.5f} pressed xmin={cap1[0][0]:.5f}",
    )

    # ---- flip cover: REVOLUTE, opens upward ----
    if r.has_flip_cover:
        jc = object_model.get_articulation("cover_hinge")
        ctx.check(
            "cover_hinge_is_revolute",
            jc.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={jc.articulation_type}",
        )
        cover = object_model.get_part("front_cover")
        c0 = ctx.part_world_aabb(cover)
        with ctx.pose({jc: 1.5}):
            c1 = ctx.part_world_aabb(cover)
        ctx.check(
            "opening_cover_raises_it",
            (c1[0][2] + c1[1][2]) / 2.0 > (c0[0][2] + c0[1][2]) / 2.0 + 0.004,
            details=f"closed zc={(c0[0][2] + c0[1][2]) / 2.0:.4f} open zc={(c1[0][2] + c1[1][2]) / 2.0:.4f}",
        )

    # ---- belt clip: hugs back, swings open ----
    clip_j = object_model.get_articulation("clip_hinge")
    ctx.check(
        "clip_hinge_revolute_0_to_25deg",
        clip_j.articulation_type == ArticulationType.REVOLUTE
        and abs(clip_j.motion_limits.upper - CLIP_OPEN) < 1e-6,
        details=f"upper={clip_j.motion_limits.upper}",
    )
    clip0 = ctx.part_world_aabb(clip)
    with ctx.pose({clip_j: CLIP_OPEN}):
        clip1 = ctx.part_world_aabb(clip)
    ctx.check(
        "clip_tip_swings_away_from_back",
        clip1[1][1] > clip0[1][1] + 0.020,
        details=f"rest ymax={clip0[1][1]:.4f} open ymax={clip1[1][1]:.4f}",
    )

    # ---- body scale + grounding ----
    body_aabb = ctx.part_world_aabb(body)
    (bx0, by0, bz0), (bx1, by1, bz1) = body_aabb
    ctx.check(
        "body_grounded_and_upright",
        abs(bz0) < 1e-3 and (bz1 - bz0) > (bx1 - bx0),
        details=f"body aabb={body_aabb}",
    )

    # ---- slot_choices recorded ----
    ctx.check(
        "slot_choices_recorded",
        tuple(object_model.meta.get("slot_choices", ())) == slot_choices_for_config(r),
        details=str(object_model.meta.get("slot_choices")),
    )

    return ctx.report()


__all__ = (
    "FieldRadioConfig",
    "ResolvedFieldRadioConfig",
    "build_field_radio",
    "build_seeded_field_radio",
    "config_from_seed",
    "resolve_config",
    "run_field_radio_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
    "with_overrides",
)
