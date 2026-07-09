"""Modular procedural template — wire_stripper (Electrical_Wiring / Wire stripper).

FORM-DOMINATED: the ③ ``mechanism_family`` slot swaps the whole part tree +
joint topology across three families (different joint graphs):

- ``manual_notch_plier`` / ``fixed_hole_gauge``  (A skeleton, meters via MM):
  ``fixed_arm``(root) + ``moving_arm`` + ``lock_latch``; joints
  ``pivot_squeeze`` REVOLUTE(+Z) + ``latch_pivot`` REVOLUTE(-Z). Tool in XY
  plane (+X nose, Y opening, Z thickness), pivot at origin; arms authored
  closed then counter-rotated ±DELTA into the open rest pose. Geometry is all
  CadQuery boolean: ``_a_poly_solid`` / ``_a_circle_solid`` /
  ``_a_cut_edge_circles`` carve real graduated notch/gauge cuts + nose
  serrations; grips are catmull-rom spline lofts + fillet (never Box).
- ``auto_selfadjust``  (B skeleton, meters direct): ``fixed_arm``(root body) +
  ``moving_arm``(crossed lever) + ``wire_stop_slider`` (+ optional
  ``gripping_jaw`` clamp part); joints ``arm_pivot`` REVOLUTE(+Z) +
  ``wire_stop_slide`` PRISMATIC(+X) (+ optional ``clamp_pinch`` REVOLUTE mimic).
  Geometry = ExtrudeGeometry/BoxGeometry/Cylinder + a spline coil spring.

Slots (registered in ``slot_choices``): ③ ``mechanism_family`` (3) ·
``handle_form`` (3, family-gated) · ``jaw_feature`` (4, family-gated) · ②
``auto_clamp_dof`` (2, auto only) · ① ``gauge_station`` N (A families only).
``palette_style`` (5 realistic colorways) drives every material.

Sources: origin A ``rec_a-…a2129614`` + origin B ``rec_an-…e60caa10`` + forks
fixed_hole_gauge / crimp_die_station / gauge_n6 / pistol_grip / auto_clamp_jaw.
The pivot is captured-pin geometry (shaft through the moving lap / crossed hub),
so the pivot joint omits MatingContract (grandfathered) and is guarded by the
flat 0.015 articulation-origin baseline + element-scoped ``allow_overlap``.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Literal, Optional

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)

__modular__ = True

# ---------------------------------------------------------------------------
# Slot vocabularies
# ---------------------------------------------------------------------------
MechanismFamily = Literal["manual_notch_plier", "fixed_hole_gauge", "auto_selfadjust"]
HandleForm = Literal["straight_plier", "angled_offset", "pistol_grip"]
JawFeature = Literal["notch_cutter", "notch_crimp_die", "gauge_hole_plate", "clamp_screw"]
AutoClampDof = Literal["abstracted", "real_clamp_joint"]
PaletteStyle = Literal["yellow_black", "red_black", "gunmetal", "blue_black", "safety_orange"]

FAMILIES: tuple[MechanismFamily, ...] = (
    "manual_notch_plier",
    "fixed_hole_gauge",
    "auto_selfadjust",
)
A_FAMILIES: tuple[MechanismFamily, ...] = ("manual_notch_plier", "fixed_hole_gauge")
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "yellow_black",
    "red_black",
    "gunmetal",
    "blue_black",
    "safety_orange",
)

# Semantic material keys used across BOTH families; every .visual() material is
# mats[<key>] so palette_style recolors the whole swept output.
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "yellow_black": {
        "steel_body": (0.14, 0.145, 0.15, 1.0),
        "steel_bright": (0.80, 0.82, 0.84, 1.0),
        "grip_main": (0.95, 0.76, 0.05, 1.0),
        "grip_dark": (0.055, 0.055, 0.06, 1.0),
        "spring": (0.24, 0.24, 0.26, 1.0),
        "hardware": (0.10, 0.10, 0.11, 1.0),
        "brass": (0.72, 0.58, 0.25, 1.0),
        "jaw": (0.14, 0.14, 0.15, 1.0),
        "slider": (0.85, 0.10, 0.09, 1.0),
    },
    "red_black": {
        "steel_body": (0.10, 0.10, 0.11, 1.0),
        "steel_bright": (0.75, 0.77, 0.80, 1.0),
        "grip_main": (0.78, 0.08, 0.07, 1.0),
        "grip_dark": (0.05, 0.05, 0.05, 1.0),
        "spring": (0.25, 0.26, 0.28, 1.0),
        "hardware": (0.10, 0.10, 0.11, 1.0),
        "brass": (0.72, 0.58, 0.25, 1.0),
        "jaw": (0.14, 0.14, 0.15, 1.0),
        "slider": (0.85, 0.10, 0.09, 1.0),
    },
    "gunmetal": {
        "steel_body": (0.28, 0.29, 0.31, 1.0),
        "steel_bright": (0.66, 0.68, 0.72, 1.0),
        "grip_main": (0.34, 0.35, 0.38, 1.0),
        "grip_dark": (0.08, 0.08, 0.09, 1.0),
        "spring": (0.30, 0.31, 0.33, 1.0),
        "hardware": (0.12, 0.12, 0.13, 1.0),
        "brass": (0.70, 0.60, 0.30, 1.0),
        "jaw": (0.18, 0.18, 0.20, 1.0),
        "slider": (0.55, 0.12, 0.10, 1.0),
    },
    "blue_black": {
        "steel_body": (0.16, 0.17, 0.19, 1.0),
        "steel_bright": (0.78, 0.80, 0.84, 1.0),
        "grip_main": (0.10, 0.24, 0.55, 1.0),
        "grip_dark": (0.05, 0.05, 0.06, 1.0),
        "spring": (0.24, 0.24, 0.26, 1.0),
        "hardware": (0.10, 0.10, 0.11, 1.0),
        "brass": (0.72, 0.58, 0.25, 1.0),
        "jaw": (0.14, 0.14, 0.15, 1.0),
        "slider": (0.90, 0.72, 0.10, 1.0),
    },
    "safety_orange": {
        "steel_body": (0.15, 0.155, 0.16, 1.0),
        "steel_bright": (0.82, 0.84, 0.86, 1.0),
        "grip_main": (0.95, 0.42, 0.10, 1.0),
        "grip_dark": (0.10, 0.10, 0.11, 1.0),
        "spring": (0.24, 0.24, 0.26, 1.0),
        "hardware": (0.08, 0.08, 0.09, 1.0),
        "brass": (0.72, 0.58, 0.25, 1.0),
        "jaw": (0.14, 0.14, 0.15, 1.0),
        "slider": (0.15, 0.15, 0.16, 1.0),
    },
}

# ---------------------------------------------------------------------------
# A-family (manual / gauge) constants — millimetre numbers scaled by `mm`.
# ---------------------------------------------------------------------------
T_STEEL = 6.0
Z_TOP = T_STEEL / 2.0
Z_BOT = -T_STEEL / 2.0
LAP_GAP = 0.30
BOSS_R = 12.0
PIN_BORE_R = 2.9
PIN_SHAFT_R = PIN_BORE_R + 0.15
PLATE_HOLE_R = 3.3
SHANK_SLOPE = 8.5 / 35.4
HOLE_CY = 7.5
CRIMP_SPECS = [(16.5, 2.6), (22.0, 2.3)]
CRIMP_DIE_SPECS = [(17.5, 2.4), (22.5, 1.9)]
MESH_TOL = 0.00015
MESH_ANG = 0.25
DELTA_BASE = 0.19

# ---------------------------------------------------------------------------
# B-family (auto) constants — meters direct.
# ---------------------------------------------------------------------------
PLATE_T = 0.003
BODY_PLATE_ZC = -0.0017
LEVER_PLATE_ZC = 0.0017
SQUEEZE_BASE = 0.10
SLIDE_MAX = 0.008
CLAMP_ANGLE = 0.04
HANDLE_ANGLE = 0.2405
PISTOL_GRIP_ANGLE = 1.15
_SHANK_X0, _SHANK_X1 = -0.0105, -0.132
_UP_Y0, _UP_Y1 = 0.0105, -0.0225
_LO_Y0, _LO_Y1 = -0.0105, -0.0385
_UP_M = (_UP_Y1 - _UP_Y0) / (_SHANK_X1 - _SHANK_X0)


@dataclass(frozen=True)
class WireStripperConfig:
    mechanism_family: Optional[MechanismFamily] = None
    handle_form: Optional[HandleForm] = None
    jaw_feature: Optional[JawFeature] = None
    auto_clamp_dof: Optional[AutoClampDof] = None
    gauge_count: Optional[int] = None
    palette_style: PaletteStyle = "yellow_black"
    open_angle_scale: float = 1.0
    overall_scale: float = 1.0
    name: str = "wire_stripper"


@dataclass(frozen=True)
class ResolvedWireStripperConfig:
    mechanism_family: MechanismFamily
    handle_form: HandleForm
    jaw_feature: JawFeature
    auto_clamp_dof: AutoClampDof
    gauge_count: int
    palette_style: PaletteStyle
    delta: float
    q_close: float
    squeeze_max: float
    mm_scale: float
    name: str

    @property
    def is_auto(self) -> bool:
        return self.mechanism_family == "auto_selfadjust"

    @property
    def is_gauge(self) -> bool:
        return self.mechanism_family == "fixed_hole_gauge"


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _pick(value, choices):
    return value if value in choices else choices[0]


# ---------------------------------------------------------------------------
# config_from_seed — procedural weighted sampling (seed 0 not special).
# ---------------------------------------------------------------------------
def config_from_seed(seed: int) -> WireStripperConfig:
    rng = random.Random(seed)
    family = rng.choices(FAMILIES, weights=[4, 2, 3], k=1)[0]
    if family == "auto_selfadjust":
        handle = rng.choices(["angled_offset", "pistol_grip"], weights=[3, 2], k=1)[0]
        jaw = "clamp_screw"
        dof = rng.choices(["abstracted", "real_clamp_joint"], weights=[3, 2], k=1)[0]
        gauge = 0
    else:
        handle = "straight_plier"
        dof = "abstracted"
        if family == "manual_notch_plier":
            jaw = rng.choices(["notch_cutter", "notch_crimp_die"], weights=[3, 2], k=1)[0]
            gauge = rng.choices([3, 4, 5, 6, 7, 8], weights=[3, 5, 4, 3, 1, 1], k=1)[0]
        else:
            jaw = "gauge_hole_plate"
            gauge = rng.choices([3, 4, 5, 6, 7], weights=[3, 4, 4, 2, 1], k=1)[0]
    return WireStripperConfig(
        mechanism_family=family,
        handle_form=handle,
        jaw_feature=jaw,
        auto_clamp_dof=dof,
        gauge_count=gauge,
        palette_style=rng.choice(PALETTE_STYLES),
        open_angle_scale=round(rng.uniform(0.90, 1.10), 4),
        overall_scale=round(rng.uniform(0.92, 1.08), 4),
        name=f"seeded_wire_stripper_{seed}",
    )


def resolve_config(config: WireStripperConfig | None = None) -> ResolvedWireStripperConfig:
    cfg = config or WireStripperConfig()
    family = _pick(cfg.mechanism_family, FAMILIES)
    palette = _pick(cfg.palette_style, PALETTE_STYLES)
    open_scale = _clamp(cfg.open_angle_scale, 0.90, 1.10)

    if family == "auto_selfadjust":
        handle = cfg.handle_form if cfg.handle_form in ("angled_offset", "pistol_grip") else "angled_offset"
        jaw = "clamp_screw"
        dof = cfg.auto_clamp_dof if cfg.auto_clamp_dof in ("abstracted", "real_clamp_joint") else "abstracted"
        gauge = 0
        squeeze_max = _clamp(SQUEEZE_BASE * open_scale, 0.09, 0.11)
        delta = 0.0
        q_close = 0.0
        mm_scale = 1.0
    else:
        handle = "straight_plier"
        dof = "abstracted"
        if family == "manual_notch_plier":
            jaw = cfg.jaw_feature if cfg.jaw_feature in ("notch_cutter", "notch_crimp_die") else "notch_cutter"
            gauge = int(_clamp(cfg.gauge_count if cfg.gauge_count is not None else 4, 3, 8))
        else:
            jaw = "gauge_hole_plate"
            gauge = int(_clamp(cfg.gauge_count if cfg.gauge_count is not None else 4, 3, 7))
        delta = _clamp(DELTA_BASE * open_scale, 0.16, 0.215)
        q_close = 2.0 * delta - 0.02
        squeeze_max = 0.0
        mm_scale = _clamp(cfg.overall_scale, 0.92, 1.08)

    return ResolvedWireStripperConfig(
        mechanism_family=family,
        handle_form=handle,
        jaw_feature=jaw,
        auto_clamp_dof=dof,
        gauge_count=gauge,
        palette_style=palette,
        delta=delta,
        q_close=q_close,
        squeeze_max=squeeze_max,
        mm_scale=mm_scale,
        name=cfg.name or "wire_stripper",
    )


def slot_choices_for_config(config) -> tuple[tuple[str, str], ...]:
    r = config if isinstance(config, ResolvedWireStripperConfig) else resolve_config(config)
    choices: list[tuple[str, str]] = [
        ("mechanism_family", r.mechanism_family),
        ("handle_form", r.handle_form),
        ("jaw_feature", r.jaw_feature),
    ]
    if r.is_auto:
        choices.append(("auto_clamp_dof", r.auto_clamp_dof))
    else:
        choices.append(("gauge_station", f"gauge_n{r.gauge_count}"))
    return tuple(choices)


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


# ---------------------------------------------------------------------------
# gauge-station multiplicity spec generators (ascending-radius invariant).
# ---------------------------------------------------------------------------
def _a_notch_specs(n: int) -> list[tuple[float, float]]:
    """N graduated stripping notches (x centre mm, cut radius mm), ascending."""
    x0, x1 = 26.0, 47.0
    r0, r1 = 0.72, 2.15
    specs = []
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0.0
        specs.append((round(x0 + t * (x1 - x0), 3), round(r0 + t * (r1 - r0), 3)))
    return specs


def _a_awg_specs(n: int) -> list[tuple[float, float]]:
    """N drilled AWG gauge holes (x centre mm, hole radius mm), ascending."""
    x0, dx = 25.0, 7.0
    r0, r1 = 0.34, 1.28
    specs = []
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0.0
        specs.append((round(x0 + i * dx, 3), round(r0 + t * (r1 - r0), 3)))
    return specs


# ---------------------------------------------------------------------------
# A-family CadQuery helpers (port of origin A + gauge/crimp forks). `mm`
# threads a uniform overall_scale so every coordinate scales together.
# ---------------------------------------------------------------------------
def _rot2(p, a):
    c, s = math.cos(a), math.sin(a)
    return (p[0] * c - p[1] * s, p[0] * s + p[1] * c)


def _a_mirror(pts):
    return [(x, -y) for (x, y) in reversed(pts)]


def _a_poly_solid(pts_mm, z0, z1, mm):
    pts = [(x * mm, y * mm) for (x, y) in pts_mm]
    return cq.Workplane("XY", origin=(0.0, 0.0, z0 * mm)).polyline(pts).close().extrude((z1 - z0) * mm)


def _a_circle_solid(cx, cy, r, z0, z1, mm):
    return cq.Workplane("XY", origin=(cx * mm, cy * mm, z0 * mm)).circle(r * mm).extrude((z1 - z0) * mm)


def _a_cut_edge_circles(solid, specs, mm):
    for (x, r) in specs:
        solid = solid.cut(_a_circle_solid(x, 0.0, r, -20.0, 20.0, mm))
    return solid


def _shank_top_y(x):
    return -1.5 + SHANK_SLOPE * (x + 12.6)


def _shank_bot_y(x):
    return -9.5 + SHANK_SLOPE * (x + 12.6)


def _teeth_pts(x0=54.0, n=8, pitch=2.75, depth=1.0):
    pts = []
    for i in range(n):
        xa = x0 + i * pitch
        pts.append((xa + pitch * 0.5, depth))
        pts.append((xa + pitch, 0.0))
    return pts


def _jaw_profile():
    pts = [(13.0, 0.0), (54.0, 0.0)]
    pts += _teeth_pts()
    pts += [
        (79.3, 0.35), (80.2, 1.1),
        (76.5, 3.0), (66.0, 6.2), (56.0, 9.8), (50.0, 12.2),
        (46.0, 14.6), (40.0, 15.2), (24.0, 15.2), (16.0, 15.0),
        (13.0, 12.0),
    ]
    return pts


def _blade_face_profile():
    return [(13.6, 0.0), (48.0, 0.0), (46.0, 13.9), (40.0, 14.5),
            (24.0, 14.5), (16.4, 14.3), (13.6, 11.6)]


def _nose_face_profile():
    pts = [(50.5, 0.0), (54.0, 0.0)]
    pts += _teeth_pts()
    pts += [(79.0, 0.35), (79.6, 0.95), (76.2, 2.7), (66.0, 5.9), (56.0, 9.4), (50.5, 11.6)]
    return pts


def _shank_profile_a():
    return [(-12.6, -1.5), (-48.0, -10.0), (-48.0, -18.0), (-12.6, -9.5)]


def _grip_profile_a():
    bottom_raw = [
        (-120.0, -20.0), (-122.5, -24.0), (-122.0, -29.0), (-118.0, -33.0),
        (-110.0, -33.8), (-90.0, -30.5), (-70.0, -26.5), (-52.0, -23.0), (-46.0, -22.0),
    ]
    bottom = sample_catmull_rom_spline_2d(bottom_raw, samples_per_segment=5)
    return [(-46.0, -8.0), (-80.0, -12.3)] + [(x, y) for (x, y) in bottom]


_SPLIT_LINE = [(-43.0, -19.3), (-70.0, -23.0), (-95.0, -27.5), (-114.0, -30.0), (-126.0, -25.0)]


def _region_below(line_pts):
    return list(line_pts) + [(-126.0, -70.0), (-43.0, -70.0)]


def _region_above(line_pts):
    return list(line_pts) + [(-126.0, 40.0), (-43.0, 40.0)]


def _collar_profile():
    return [
        (-15.0, _shank_top_y(-15.0) + 1.0),
        (-21.0, _shank_top_y(-21.0) + 1.0),
        (-21.0, _shank_bot_y(-21.0) - 1.0),
        (-15.0, _shank_bot_y(-15.0) - 1.0),
    ]


def _gauge_plate_profile():
    return [
        (13.0, 0.0), (72.0, 0.0), (76.0, 1.5), (77.5, 4.0), (77.5, 8.0),
        (76.0, 11.0), (72.0, 14.0), (50.0, 15.2), (24.0, 15.2), (16.0, 15.0), (13.0, 12.0),
    ]


def _pressure_jaw_profile():
    return [
        (13.0, 0.0), (72.0, 0.0), (75.0, -1.0), (76.0, -3.5), (76.0, -7.5),
        (75.0, -10.0), (72.0, -12.5), (50.0, -13.5), (24.0, -13.5), (16.0, -13.0), (13.0, -10.5),
    ]


def _a_build_jaw_plate(cut_specs, mm):
    solid = _a_poly_solid(_jaw_profile(), Z_BOT, Z_TOP, mm)
    v0 = solid.val().Volume()
    solid = _a_cut_edge_circles(solid, cut_specs, mm)
    v1 = solid.val().Volume()
    return solid, v0, v1


def _a_build_blade_face(cut_specs, mm):
    solid = _a_poly_solid(_blade_face_profile(), Z_TOP - 0.2, Z_TOP + 0.5, mm)
    return _a_cut_edge_circles(solid, cut_specs, mm)


def _a_build_nose_face(mm):
    return _a_poly_solid(_nose_face_profile(), Z_TOP - 0.2, Z_TOP + 0.5, mm)


def _a_build_gauge_plate(specs, mm):
    solid = _a_poly_solid(_gauge_plate_profile(), Z_BOT, Z_TOP, mm)
    v0 = solid.val().Volume()
    for (x, r) in specs:
        solid = solid.cut(_a_circle_solid(x, HOLE_CY, r, -20.0, 20.0, mm))
    v1 = solid.val().Volume()
    return solid, v0, v1


def _a_build_gauge_face(specs, mm):
    solid = _a_poly_solid(_gauge_plate_profile(), Z_TOP - 0.2, Z_TOP + 0.5, mm)
    for (x, r) in specs:
        solid = solid.cut(_a_circle_solid(x, HOLE_CY, r, -20.0, 20.0, mm))
    return solid


def _a_build_pressure_jaw(mm):
    return _a_poly_solid(_pressure_jaw_profile(), Z_BOT, Z_TOP, mm)


def _a_build_pressure_face(mm):
    return _a_poly_solid(_pressure_jaw_profile(), Z_TOP - 0.2, Z_TOP + 0.5, mm)


def _a_build_crimp_die_block(mirrored, mm):
    x0, x1 = 14.5, 25.5
    if not mirrored:
        y0, y1 = 0.0, 12.0
    else:
        y0, y1 = -12.0, 0.0
    z0, z1 = Z_TOP - 0.2, Z_TOP + 1.0
    block = _a_poly_solid([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], z0, z1, mm)
    for (cx, r) in CRIMP_DIE_SPECS:
        block = block.cut(_a_circle_solid(cx, 0.0, r, z0 - 1.0, z1 + 1.0, mm))
    try:
        block = block.edges(">Z").fillet(0.4 * mm)
    except Exception:
        try:
            block = block.edges(">Z").chamfer(0.3 * mm)
        except Exception:
            pass
    return block


def _a_build_fixed_lap(mm):
    z0, z1 = Z_BOT, -LAP_GAP / 2.0
    disc = _a_circle_solid(0.0, 0.0, BOSS_R, z0, z1, mm)
    jaw_tab = _a_poly_solid([(9.0, 0.8), (16.0, 0.8), (16.0, 13.5), (9.0, 13.5)], z0, z1, mm)
    shank_tab = _a_poly_solid([(-16.0, -9.5), (-9.0, -9.5), (-9.0, -0.8), (-16.0, -0.8)], z0, z1, mm)
    return disc.union(jaw_tab).union(shank_tab)


def _a_build_moving_lap(mm):
    z0, z1 = LAP_GAP / 2.0, Z_TOP
    disc = _a_circle_solid(0.0, 0.0, BOSS_R, z0, z1, mm)
    disc = disc.cut(_a_circle_solid(0.0, 0.0, PIN_BORE_R, -20.0, 20.0, mm))
    jaw_tab = _a_poly_solid([(9.0, -13.5), (16.0, -13.5), (16.0, -0.8), (9.0, -0.8)], z0, z1, mm)
    shank_tab = _a_poly_solid([(-16.0, 0.8), (-9.0, 0.8), (-9.0, 9.5), (-16.0, 9.5)], z0, z1, mm)
    return disc.union(jaw_tab).union(shank_tab)


def _a_soften(solid, r_mm, mm):
    for sel in (">Z", "<Z"):
        try:
            solid = solid.edges(sel).fillet(r_mm * mm)
        except Exception:
            try:
                solid = solid.edges(sel).chamfer(0.6 * r_mm * mm)
            except Exception:
                pass
    return solid


def _a_build_grip_pair(mirrored, mm):
    grip_pts = _grip_profile_a()
    split = list(_SPLIT_LINE)
    split_low = [(x, y - 1.0) for (x, y) in split]
    if mirrored:
        grip_pts = _a_mirror(grip_pts)
        below = _a_poly_solid(_a_mirror(_region_below(split_low)), -9.0, 9.0, mm)
        above = _a_poly_solid(_a_mirror(_region_above(split)), -9.0, 9.0, mm)
    else:
        below = _a_poly_solid(_region_below(split_low), -9.0, 9.0, mm)
        above = _a_poly_solid(_region_above(split), -9.0, 9.0, mm)
    yellow = _a_poly_solid(grip_pts, -7.0, 7.0, mm).cut(below)
    black = _a_poly_solid(grip_pts, -7.6, 7.6, mm).cut(above)
    return _a_soften(yellow, 1.8, mm), _a_soften(black, 1.8, mm)


def _a_build_pivot_plate(mm):
    plate = cq.Workplane("XY", origin=(0.0, 0.0, 2.9 * mm)).ellipse(11.0 * mm, 8.0 * mm).extrude(0.5 * mm)
    plate = plate.cut(_a_circle_solid(0.0, 0.0, PLATE_HOLE_R, -20.0, 20.0, mm))
    slot = _a_poly_solid([(4.3, -1.2), (8.3, -1.2), (8.3, 1.2), (4.3, 1.2)], -20.0, 20.0, mm)
    slot = slot.union(_a_circle_solid(4.3, 0.0, 1.2, -20.0, 20.0, mm))
    slot = slot.union(_a_circle_solid(8.3, 0.0, 1.2, -20.0, 20.0, mm))
    return plate.cut(slot)


def _a_build_pivot_pin(mm):
    flange = CylinderGeometry(5.0 * mm, 1.4 * mm, radial_segments=40)
    flange.translate(0.0, 0.0, -3.5 * mm)
    shaft = CylinderGeometry(PIN_SHAFT_R * mm, 7.62 * mm, radial_segments=40)
    shaft.translate(0.0, 0.0, -0.19 * mm)
    head = CylinderGeometry(4.6 * mm, 1.4 * mm, radial_segments=40)
    head.translate(0.0, 0.0, 4.2 * mm)
    return flange.merge(shaft).merge(head)


def _a_spring_anchor(delta):
    return _rot2((-40.0, _shank_top_y(-40.0)), delta)


def _a_build_spring(delta, mm):
    bx, by = _a_spring_anchor(delta)
    seat = CylinderGeometry(5.5 * mm, 4.5 * mm, radial_segments=32)
    seat.rotate_x(math.pi / 2.0)
    seat.translate(bx * mm, (by - 0.75) * mm, 0.0)
    coil_r, wire_r, turns = 4.2, 1.05, 6.5
    # coil length tracks the opening so its free end still reaches the moving handle.
    s0, s1 = 1.0, 26.5 * (delta / DELTA_BASE)
    n = int(turns * 16)
    pts = []
    for i in range(n + 1):
        t = i / n
        s = s0 + t * (s1 - s0)
        th = 2.0 * math.pi * turns * t
        pts.append(((bx + coil_r * math.cos(th)) * mm, (by + s) * mm, (coil_r * math.sin(th)) * mm))
    coil = tube_from_spline_points(pts, radius=wire_r * mm, samples_per_segment=2, radial_segments=12, cap_ends=True)
    return seat.merge(coil)


def _a_build_latch(mm):
    pad = cq.Workplane("XY").box(9.0 * mm, 6.7 * mm, 3.5 * mm)
    try:
        pad = pad.edges("|Z").fillet(1.5 * mm)
    except Exception:
        pass
    pad = pad.translate((4.0 * mm, -0.35 * mm, 1.75 * mm))
    hook = cq.Workplane("XY").box(4.0 * mm, 4.5 * mm, 4.5 * mm).translate((3.5 * mm, -5.25 * mm, 0.25 * mm))
    latch = pad.union(hook)
    for cx in (1.8, 4.0, 6.2):
        rib = cq.Workplane("XY").box(0.9 * mm, 6.3 * mm, 0.9 * mm).translate((cx * mm, -0.35 * mm, 3.65 * mm))
        latch = latch.union(rib)
    return latch


def _acq(solid, mesh_name):
    return mesh_from_cadquery(solid, mesh_name, tolerance=MESH_TOL, angular_tolerance=MESH_ANG)


# ---------------------------------------------------------------------------
# A-family assembly (manual_notch_plier / fixed_hole_gauge).
# ---------------------------------------------------------------------------
def _build_A_family(model: ArticulatedObject, r: ResolvedWireStripperConfig, mats) -> None:
    mm = 0.001 * r.mm_scale
    delta, q_close = r.delta, r.q_close
    open_fixed = Origin(rpy=(0.0, 0.0, delta))
    open_moving = Origin(rpy=(0.0, 0.0, -delta))

    fixed = model.part("fixed_arm")

    if r.is_gauge:
        specs = _a_awg_specs(r.gauge_count)
        gauge, v0, v1 = _a_build_gauge_plate(specs, mm)
        removed = v0 - v1
        expected = sum(math.pi * (rr * mm) ** 2 * (T_STEEL * mm) for _, rr in specs)
        fixed.visual(_acq(gauge, "fixed_gauge_plate"), origin=open_fixed, material=mats["steel_body"], name="gauge_plate")
        fixed.visual(_acq(_a_build_gauge_face(specs, mm), "fixed_gauge_face"), origin=open_fixed, material=mats["steel_bright"], name="gauge_face")
    else:
        specs = _a_notch_specs(r.gauge_count)
        cut = (CRIMP_SPECS + specs) if r.jaw_feature == "notch_cutter" else list(specs)
        jaw, v0, v1 = _a_build_jaw_plate(cut, mm)
        removed = v0 - v1
        expected = sum(0.5 * math.pi * (rr * mm) ** 2 * (T_STEEL * mm) for _, rr in cut)
        fixed.visual(_acq(jaw, "fixed_jaw_plate"), origin=open_fixed, material=mats["steel_body"], name="jaw_plate")
        fixed.visual(_acq(_a_build_blade_face(cut, mm), "fixed_blade_face"), origin=open_fixed, material=mats["steel_bright"], name="blade_face")
        fixed.visual(_acq(_a_build_nose_face(mm), "fixed_nose_face"), origin=open_fixed, material=mats["steel_bright"], name="nose_face")
        if r.jaw_feature == "notch_crimp_die":
            fixed.visual(_acq(_a_build_crimp_die_block(False, mm), "fixed_crimp_die"), origin=open_fixed, material=mats["steel_bright"], name="crimp_die")

    fixed.visual(_acq(_a_build_fixed_lap(mm), "fixed_lap_plate"), origin=open_fixed, material=mats["steel_body"], name="lap_plate")
    fixed.visual(_acq(_a_poly_solid(_shank_profile_a(), Z_BOT, Z_TOP, mm), "fixed_arm_shank"), origin=open_fixed, material=mats["steel_body"], name="arm_shank")
    grip_y, grip_b = _a_build_grip_pair(False, mm)
    fixed.visual(_acq(grip_y, "fixed_grip_body"), origin=open_fixed, material=mats["grip_main"], name="grip_body")
    fixed.visual(_acq(grip_b, "fixed_grip_overmold"), origin=open_fixed, material=mats["grip_dark"], name="grip_overmold")
    fixed.visual(_acq(_a_poly_solid(_collar_profile(), -4.0, 4.0, mm), "fixed_grip_collar"), origin=open_fixed, material=mats["grip_main"], name="grip_collar")
    fixed.visual(mesh_from_geometry(_a_build_pivot_pin(mm), "pivot_pin"), origin=open_fixed, material=mats["hardware"], name="pivot_pin")
    fixed.visual(mesh_from_geometry(_a_build_spring(delta, mm), "handle_spring"), material=mats["spring"], name="handle_spring")

    moving = model.part("moving_arm")
    if r.is_gauge:
        moving.visual(_acq(_a_build_pressure_jaw(mm), "moving_pressure_jaw"), origin=open_moving, material=mats["steel_body"], name="pressure_jaw")
        moving.visual(_acq(_a_build_pressure_face(mm), "moving_pressure_face"), origin=open_moving, material=mats["steel_bright"], name="pressure_face")
    else:
        jaw_m = _a_poly_solid(_a_mirror(_jaw_profile()), Z_BOT, Z_TOP, mm)
        jaw_m = _a_cut_edge_circles(jaw_m, cut, mm)
        moving.visual(_acq(jaw_m, "moving_jaw_plate"), origin=open_moving, material=mats["steel_body"], name="jaw_plate")
        blade_m = _a_poly_solid(_a_mirror(_blade_face_profile()), Z_TOP - 0.2, Z_TOP + 0.5, mm)
        blade_m = _a_cut_edge_circles(blade_m, cut, mm)
        moving.visual(_acq(blade_m, "moving_blade_face"), origin=open_moving, material=mats["steel_bright"], name="blade_face")
        moving.visual(_acq(_a_poly_solid(_a_mirror(_nose_face_profile()), Z_TOP - 0.2, Z_TOP + 0.5, mm), "moving_nose_face"), origin=open_moving, material=mats["steel_bright"], name="nose_face")
        if r.jaw_feature == "notch_crimp_die":
            moving.visual(_acq(_a_build_crimp_die_block(True, mm), "moving_crimp_die"), origin=open_moving, material=mats["steel_bright"], name="crimp_die")

    moving.visual(_acq(_a_build_moving_lap(mm), "moving_lap_plate"), origin=open_moving, material=mats["steel_body"], name="lap_plate")
    moving.visual(_acq(_a_poly_solid(_a_mirror(_shank_profile_a()), Z_BOT, Z_TOP, mm), "moving_arm_shank"), origin=open_moving, material=mats["steel_body"], name="arm_shank")
    grip_ym, grip_bm = _a_build_grip_pair(True, mm)
    moving.visual(_acq(grip_ym, "moving_grip_body"), origin=open_moving, material=mats["grip_main"], name="grip_body")
    moving.visual(_acq(grip_bm, "moving_grip_overmold"), origin=open_moving, material=mats["grip_dark"], name="grip_overmold")
    moving.visual(_acq(_a_poly_solid(_a_mirror(_collar_profile()), -4.0, 4.0, mm), "moving_grip_collar"), origin=open_moving, material=mats["grip_main"], name="grip_collar")
    moving.visual(_acq(_a_build_pivot_plate(mm), "pivot_plate"), origin=open_moving, material=mats["steel_bright"], name="pivot_plate")

    latch = model.part("lock_latch")
    latch.visual(_acq(_a_build_latch(mm), "latch_body"), material=mats["hardware"], name="latch_body")

    model.articulation(
        "pivot_squeeze", ArticulationType.REVOLUTE, parent=fixed, child=moving,
        origin=Origin(xyz=(0.0, 0.0, 0.0)), axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=4.0, lower=0.0, upper=q_close),
    )
    latch_xy = _rot2((-31.0, 5.5), -delta)
    model.articulation(
        "latch_pivot", ArticulationType.REVOLUTE, parent=moving, child=latch,
        origin=Origin(xyz=(latch_xy[0] * mm, latch_xy[1] * mm, Z_TOP * mm), rpy=(0.0, 0.0, -delta)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=0.45),
    )

    model.meta["jaw_removed_volume"] = removed
    model.meta["jaw_expected_volume"] = expected
    model.meta["gauge_specs"] = specs


# ---------------------------------------------------------------------------
# B-family helpers (port of origin B + pistol_grip / auto_clamp forks).
# ---------------------------------------------------------------------------
def _b_y_upper(x):
    return _UP_Y0 + (x - _SHANK_X0) * _UP_M


def _b_ccw(points):
    area = 0.0
    n = len(points)
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        area += x0 * y1 - x1 * y0
    return points if area > 0.0 else list(reversed(points))


def _b_mirror(points):
    return [(x, -y) for (x, y) in points]


def _b_circle(radius, n=48):
    return [(radius * math.cos(2.0 * math.pi * i / n), radius * math.sin(2.0 * math.pi * i / n)) for i in range(n)]


def _b_head_half_profile():
    return [(0.078, 0.001), (0.078, 0.0245), (0.048, 0.029), (0.016, 0.0215), (0.011, 0.0165), (0.011, 0.001)]


def _b_shank_profile():
    top_edge = [(_SHANK_X0, _UP_Y0), (-0.016, _b_y_upper(-0.016))]
    pitch = 0.0072
    for i in range(5):
        x_dip = -0.0196 - i * pitch
        x_ret = -0.0232 - i * pitch
        top_edge.append((x_dip, _b_y_upper(x_dip) - 0.0045))
        top_edge.append((x_ret, _b_y_upper(x_ret)))
    top_edge.append((_SHANK_X1, _UP_Y1))
    poly = [(_SHANK_X1, _LO_Y1), (_SHANK_X0, _LO_Y0)]
    poly.extend(top_edge)
    return poly


def _b_plate_mesh(mirror):
    head = _b_head_half_profile()
    shank = _b_shank_profile()
    if mirror:
        head = _b_mirror(head)
        shank = _b_mirror(shank)
    zc = LEVER_PLATE_ZC if mirror else BODY_PLATE_ZC
    plate = ExtrudeGeometry(_b_ccw(head), PLATE_T, cap=True, center=True)
    boss_t = PLATE_T + 0.00004
    if mirror:
        boss = ExtrudeWithHolesGeometry(_b_ccw(_b_circle(0.0165)), [_b_ccw(_b_circle(0.0105))], boss_t, cap=True, center=True)
    else:
        boss = CylinderGeometry(0.0165, boss_t, radial_segments=48)
    plate.merge(boss)
    plate.merge(ExtrudeGeometry(_b_ccw(shank), PLATE_T, cap=True, center=True))
    plate.translate(0.0, 0.0, zc)
    return plate


def _b_grip_slab(length, width, thickness, center, angle, radius=0.005):
    slab = ExtrudeGeometry(_b_ccw(rounded_rect_profile(length, width, radius)), thickness, cap=True, center=True)
    slab.rotate_z(angle)
    slab.translate(*center)
    return slab


def _b_spring_mesh():
    x0, x1 = 0.016, 0.0225
    turns = 4.0
    n = 40
    pts = []
    for i in range(n + 1):
        t = i / n
        ang = 2.0 * math.pi * turns * t
        pts.append((x0 + (x1 - x0) * t, 0.0105 + 0.0022 * math.cos(ang), 0.0045 + 0.0022 * math.sin(ang)))
    return tube_from_spline_points(pts, radius=0.0008, samples_per_segment=2, radial_segments=8, cap_ends=True)


def _b_emit_grips(part, side, handle_form, mats):
    is_body = side == "body"
    sy = -1.0 if is_body else 1.0
    za = -1.0 if is_body else 1.0
    if handle_form == "pistol_grip":
        ang = -PISTOL_GRIP_ANGLE if is_body else PISTOL_GRIP_ANGLE
        part.visual(mesh_from_geometry(_b_grip_slab(0.058, 0.022, 0.016, (-0.078, sy * 0.045, za * 0.0016), ang, radius=0.006), f"{side}_grip"),
                    material=mats["grip_main"], name="grip_body")
        part.visual(mesh_from_geometry(_b_grip_slab(0.036, 0.009, 0.0022, (-0.078, sy * 0.045, za * 0.0106), ang, radius=0.003), f"{side}_grip_inlay"),
                    material=mats["grip_dark"], name="grip_inlay")
        part.visual(mesh_from_geometry(_b_grip_slab(0.025, 0.028, 0.017, (-0.065, sy * 0.072, za * 0.0016), ang, radius=0.011), f"{side}_palm_swell"),
                    material=mats["grip_dark"], name="grip_tip")
    else:  # angled_offset
        ang = HANDLE_ANGLE if is_body else -HANDLE_ANGLE
        part.visual(mesh_from_geometry(_b_grip_slab(0.088, 0.0165, 0.0118, (-0.0925, sy * 0.0247, za * 0.0016), ang), f"{side}_grip"),
                    material=mats["grip_main"], name="grip_body")
        part.visual(mesh_from_geometry(_b_grip_slab(0.055, 0.0075, 0.0020, (-0.0925, sy * 0.0247, za * 0.0075), ang, radius=0.003), f"{side}_grip_inlay"),
                    material=mats["grip_dark"], name="grip_inlay")
        part.visual(mesh_from_geometry(_b_grip_slab(0.020, 0.0185, 0.0126, (-0.1292, sy * 0.0334, za * 0.0016), ang), f"{side}_grip_tip"),
                    material=mats["grip_dark"], name="grip_tip")


# ---------------------------------------------------------------------------
# B-family assembly (auto_selfadjust).
# ---------------------------------------------------------------------------
def _build_B_family(model: ArticulatedObject, r: ResolvedWireStripperConfig, mats) -> None:
    squeeze = r.squeeze_max
    real_clamp = r.auto_clamp_dof == "real_clamp_joint"

    body = model.part("fixed_arm")
    body.visual(mesh_from_geometry(_b_plate_mesh(False), "body_plate"), material=mats["steel_body"], name="body_plate")

    # outboard guide riser (carries / straddles the moving lever plate)
    riser = BoxGeometry((0.030, 0.009, 0.0051)).translate(0.061, 0.0195, 0.00195)
    body.visual(mesh_from_geometry(riser, "gripping_jaw_riser"), material=mats["jaw"], name="gripping_jaw_riser")

    if not real_clamp:
        jaw = BoxGeometry((0.030, 0.0205, 0.0100)).translate(0.061, 0.013750, 0.0090)
        body.visual(mesh_from_geometry(jaw, "gripping_jaw"), material=mats["jaw"], name="gripping_jaw")
        pad = BoxGeometry((0.024, 0.0037, 0.006)).translate(0.062, 0.002650, 0.0090)
        body.visual(mesh_from_geometry(pad, "gripping_jaw_pad"), material=mats["spring"], name="gripping_jaw_pad")
        body.visual(Cylinder(radius=0.0055, length=0.0050), origin=Origin(xyz=(0.052, 0.0145, 0.0160)), material=mats["brass"], name="tension_screw_head")
        body.visual(Cylinder(radius=0.0038, length=0.0025), origin=Origin(xyz=(0.052, 0.0145, 0.01975)), material=mats["brass"], name="tension_screw_tip")

    rail = BoxGeometry((0.024, 0.004, 0.0032)).translate(0.032, 0.0105, 0.0010)
    body.visual(mesh_from_geometry(rail, "slider_rail"), material=mats["spring"], name="slider_rail")
    body.visual(Cylinder(radius=0.0015, length=0.0081), origin=Origin(xyz=(0.0155, 0.0115, 0.00345)), material=mats["steel_bright"], name="spring_post")
    body.visual(mesh_from_geometry(_b_spring_mesh(), "stop_spring"), material=mats["spring"], name="stop_spring")
    body.visual(Cylinder(radius=0.0090, length=0.0120), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=mats["steel_bright"], name="pivot_shaft")
    body.visual(Cylinder(radius=0.0135, length=0.0030), origin=Origin(xyz=(0.0, 0.0, 0.0053)), material=mats["steel_bright"], name="pivot_button_cap")
    body.visual(Cylinder(radius=0.0135, length=0.0030), origin=Origin(xyz=(0.0, 0.0, -0.0053)), material=mats["steel_bright"], name="pivot_rear_cap")
    body.visual(Cylinder(radius=0.0025, length=0.0016), origin=Origin(xyz=(-0.058, -0.012, 0.0002)), material=mats["steel_bright"], name="shank_rivet")
    _b_emit_grips(body, "body", r.handle_form, mats)

    lever = model.part("moving_arm")
    lever.visual(mesh_from_geometry(_b_plate_mesh(True), "lever_plate"), material=mats["steel_body"], name="lever_plate")
    s_jaw = BoxGeometry((0.030, 0.0205, 0.0112)).translate(0.061, -0.013750, 0.0084)
    s_jaw.rotate_z(-squeeze)
    lever.visual(mesh_from_geometry(s_jaw, "stripping_jaw"), material=mats["jaw"], name="stripping_jaw")
    s_pad = BoxGeometry((0.024, 0.0037, 0.006)).translate(0.062, -0.002650, 0.0090)
    s_pad.rotate_z(-squeeze)
    lever.visual(mesh_from_geometry(s_pad, "stripping_jaw_pad"), material=mats["spring"], name="stripping_jaw_pad")
    for i, (rx, ry) in enumerate([(0.024, -0.019), (0.032, -0.020)]):
        lever.visual(Cylinder(radius=0.0035, length=0.0022), origin=Origin(xyz=(rx, ry, 0.0029)), material=mats["steel_bright"], name=f"head_rivet_{i}")
    lever.visual(Cylinder(radius=0.0025, length=0.0016), origin=Origin(xyz=(-0.058, 0.012, 0.0036)), material=mats["steel_bright"], name="shank_rivet")
    _b_emit_grips(lever, "lever", r.handle_form, mats)

    slider = model.part("wire_stop_slider")
    s_block = BoxGeometry((0.012, 0.012, 0.0062)).translate(0.0, 0.0, 0.0056)
    slider.visual(mesh_from_geometry(s_block, "slider_body"), material=mats["slider"], name="slider_body")
    s_tab = BoxGeometry((0.003, 0.012, 0.0055)).translate(0.0045, 0.0, 0.01075)
    slider.visual(mesh_from_geometry(s_tab, "slider_tab"), material=mats["slider"], name="slider_tab")
    for sign, tag in ((-1.0, "inner"), (1.0, "outer")):
        skirt = BoxGeometry((0.012, 0.0036, 0.0026)).translate(0.0, sign * 0.0042, 0.0018)
        slider.visual(mesh_from_geometry(skirt, f"slider_skirt_{tag}"), material=mats["slider"], name=f"slider_skirt_{tag}")

    if real_clamp:
        pin = (0.046, 0.019, 0.004)
        gj = model.part("gripping_jaw")
        jaw_block = BoxGeometry((0.030, 0.0205, 0.0100)).translate(0.061 - pin[0], 0.01375 - pin[1], 0.0090 - pin[2])
        gj.visual(mesh_from_geometry(jaw_block, "gripping_jaw"), material=mats["jaw"], name="gripping_jaw")
        jaw_pad = BoxGeometry((0.024, 0.0037, 0.006)).translate(0.062 - pin[0], 0.00265 - pin[1], 0.0090 - pin[2])
        gj.visual(mesh_from_geometry(jaw_pad, "gripping_jaw_pad"), material=mats["spring"], name="gripping_jaw_pad")
        gj.visual(Cylinder(radius=0.0055, length=0.0050), origin=Origin(xyz=(0.052 - pin[0], 0.0145 - pin[1], 0.0160 - pin[2])), material=mats["brass"], name="tension_screw_head")
        gj.visual(Cylinder(radius=0.0038, length=0.0025), origin=Origin(xyz=(0.052 - pin[0], 0.0145 - pin[1], 0.01975 - pin[2])), material=mats["brass"], name="tension_screw_tip")

    model.articulation(
        "arm_pivot", ArticulationType.REVOLUTE, parent=body, child=lever,
        origin=Origin(xyz=(0.0, 0.0, 0.0)), axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.0, lower=0.0, upper=squeeze),
    )
    model.articulation(
        "wire_stop_slide", ArticulationType.PRISMATIC, parent=body, child=slider,
        origin=Origin(xyz=(0.030, 0.0105, 0.0)), axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.05, lower=0.0, upper=SLIDE_MAX),
    )
    if real_clamp:
        model.articulation(
            "clamp_pinch", ArticulationType.REVOLUTE, parent=body, child=gj,
            origin=Origin(xyz=pin), axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=CLAMP_ANGLE),
            mimic=Mimic(joint="arm_pivot", multiplier=CLAMP_ANGLE / squeeze, offset=0.0),
        )


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_wire_stripper(config: WireStripperConfig | None = None, *, assets: AssetContext | None = None) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=r.name, assets=assets)
    palette = PALETTES[r.palette_style]
    mats = {key: model.material(f"ws_{key}_{r.palette_style}", rgba=rgba) for key, rgba in palette.items()}

    if r.is_auto:
        _build_B_family(model, r, mats)
    else:
        _build_A_family(model, r, mats)

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_wire_stripper(seed: int, *, assets: AssetContext | None = None) -> ArticulatedObject:
    return build_wire_stripper(config_from_seed(seed), assets=assets)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def _aabb_center(aabb):
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def _run_A_tests(object_model, r, ctx):
    fixed = object_model.get_part("fixed_arm")
    moving = object_model.get_part("moving_arm")
    latch = object_model.get_part("lock_latch")
    squeeze = object_model.get_articulation("pivot_squeeze")
    latch_joint = object_model.get_articulation("latch_pivot")

    # Intentional captured-pin / spring / latch overlaps (rest pose is clean).
    ctx.allow_overlap(fixed, moving, elem_a="handle_spring", elem_b="arm_shank",
                      reason="Rigid coil-spring proxy is compressed by the moving handle when squeezed.")
    ctx.allow_overlap(fixed, moving, elem_a="handle_spring", elem_b="grip_body",
                      reason="Spring free end meets the moving grip when the handles are squeezed shut.")
    ctx.allow_overlap(latch, fixed, elem_a="latch_body", elem_b="arm_shank",
                      reason="Lock latch hook engages the opposite handle when flipped while closed.")
    ctx.allow_overlap(fixed, moving, elem_a="pivot_pin", elem_b="lap_plate",
                      reason="Captured pivot pin: axisymmetric bushing fit inside the moving lap bore.")

    aabbs = [ctx.part_world_aabb(p) for p in (fixed, moving, latch)]
    lo = [min(a[0][i] for a in aabbs) for i in range(3)]
    hi = [max(a[1][i] for a in aabbs) for i in range(3)]
    length, width, thick = hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]
    ctx.check("overall length plausible", 0.160 <= length <= 0.235, details=f"length={length:.4f}")
    ctx.check("open-tool width plausible", 0.060 <= width <= 0.150, details=f"width={width:.4f}")
    ctx.check("tool thickness plausible", 0.010 <= thick <= 0.024, details=f"thick={thick:.4f}")

    ctx.expect_contact(fixed, moving, contact_tol=8e-4, name="arms mate at the lap joint pivot")
    ctx.expect_overlap(fixed, moving, axes="xy", elem_a="pivot_pin", elem_b="lap_plate", min_overlap=0.003,
                       name="pivot pin passes through the moving lap plate")

    lim = squeeze.motion_limits
    ctx.check("squeeze REVOLUTE +Z with range",
              squeeze.articulation_type == ArticulationType.REVOLUTE
              and tuple(round(abs(c), 3) for c in squeeze.axis) == (0.0, 0.0, 1.0)
              and lim is not None and abs(lim.lower) < 1e-9 and lim.upper > lim.lower,
              details=f"type={squeeze.articulation_type} axis={tuple(round(c,3) for c in squeeze.axis)}")

    if r.is_gauge:
        pos_elem = neg_elem = None
        open_a, open_b = "gauge_plate", "pressure_jaw"
    else:
        open_a = open_b = "nose_face"
    ctx.expect_gap(fixed, moving, axis="y", positive_elem=open_a, negative_elem=open_b,
                   min_gap=0.004, max_gap=0.06, name="jaws are open at rest")

    rest_center = _aabb_center(ctx.part_world_aabb(moving))
    with ctx.pose({squeeze: r.q_close}):
        ctx.expect_contact(fixed, moving, elem_a=open_a, elem_b=open_b, contact_tol=0.004,
                           name="jaws nearly meet when squeezed")
        closed_center = _aabb_center(ctx.part_world_aabb(moving))
    disp = math.dist(rest_center, closed_center)
    ctx.check("squeeze visibly swings the moving arm", disp >= 0.004, details=f"disp={disp:.4f}")

    specs = object_model.meta.get("gauge_specs", [])
    ctx.check("gauge stations ascending radius",
              len(specs) == r.gauge_count and all(specs[i][1] < specs[i + 1][1] for i in range(len(specs) - 1)),
              details=f"radii={[rr for _, rr in specs]}")
    removed = object_model.meta.get("jaw_removed_volume", 0.0)
    expected = object_model.meta.get("jaw_expected_volume", 0.0)
    ctx.check("gauge/notch cuts removed real material",
              0.55 * expected <= removed <= 1.45 * expected,
              details=f"removed={removed:.3e} expected~{expected:.3e}")

    ctx.expect_contact(fixed, moving, elem_a="handle_spring", contact_tol=0.008,
                       name="spring free end reaches near the moving handle")

    ctx.expect_contact(latch, moving, contact_tol=2e-3, name="latch seats on the moving handle face")
    latch_rest = _aabb_center(ctx.part_world_aabb(latch))
    with ctx.pose({latch_joint: 0.45}):
        latch_flip = _aabb_center(ctx.part_world_aabb(latch))
    ctx.check("lock latch flips on its pivot", math.dist(latch_rest, latch_flip) >= 0.0010,
              details=f"latch disp={math.dist(latch_rest, latch_flip):.5f}")

    if r.jaw_feature == "notch_crimp_die":
        ctx.check("crimp-die station present on both arms",
                  fixed.get_visual("crimp_die") is not None and moving.get_visual("crimp_die") is not None,
                  details="dedicated crimp_die visuals expected")


def _run_B_tests(object_model, r, ctx):
    body = object_model.get_part("fixed_arm")
    lever = object_model.get_part("moving_arm")
    slider = object_model.get_part("wire_stop_slider")
    pivot = object_model.get_articulation("arm_pivot")
    slide = object_model.get_articulation("wire_stop_slide")
    real_clamp = r.auto_clamp_dof == "real_clamp_joint"
    grip_jaw_part = object_model.get_part("gripping_jaw") if real_clamp else body

    ctx.allow_overlap(slider, body, elem_a="slider_body", elem_b="slider_rail",
                      reason="Slider seats 0.1mm into the rail top (captured sliding fit of the wire-length stop).")
    if real_clamp:
        ctx.allow_overlap(grip_jaw_part, body, elem_a="gripping_jaw", elem_b="gripping_jaw_riser",
                          reason="Gripping jaw seats into the riser top (captured revolute guide fit of the self-adjusting clamp).")

    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for part in object_model.parts:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            continue
        for k in range(3):
            lo[k] = min(lo[k], aabb[0][k])
            hi[k] = max(hi[k], aabb[1][k])
    ext = [hi[k] - lo[k] for k in range(3)]
    ctx.check("tool envelope plausible for a hand stripper",
              0.19 <= ext[0] <= 0.24 and 0.06 <= ext[1] <= 0.19 and 0.018 <= ext[2] <= 0.04,
              details=f"extents={ext}")

    ctx.check("arm_pivot REVOLUTE +Z with range",
              pivot.articulation_type == ArticulationType.REVOLUTE
              and tuple(round(abs(c), 3) for c in pivot.axis) == (0.0, 0.0, 1.0)
              and pivot.motion_limits is not None and pivot.motion_limits.upper > pivot.motion_limits.lower,
              details=f"type={pivot.articulation_type}")
    ctx.check("wire_stop_slide PRISMATIC +X",
              slide.articulation_type == ArticulationType.PRISMATIC
              and tuple(round(abs(c), 3) for c in slide.axis) == (1.0, 0.0, 0.0),
              details=f"type={slide.articulation_type} axis={tuple(round(c,3) for c in slide.axis)}")

    rest_aabb = ctx.part_world_aabb(lever)
    rest_c = [(rest_aabb[0][k] + rest_aabb[1][k]) * 0.5 for k in range(3)]
    rest_grip = ctx.part_element_world_aabb(lever, elem="grip_body")
    rest_grip_cy = (rest_grip[0][1] + rest_grip[1][1]) * 0.5

    ctx.expect_gap(grip_jaw_part, lever, axis="y", positive_elem="gripping_jaw_pad", negative_elem="stripping_jaw_pad",
                   min_gap=0.003, name="jaw wire slot is open at rest")
    with ctx.pose({pivot: r.squeeze_max}):
        sq_aabb = ctx.part_world_aabb(lever)
        sq_c = [(sq_aabb[0][k] + sq_aabb[1][k]) * 0.5 for k in range(3)]
        ctx.check("squeezing rotates the lever arm", math.dist(rest_c, sq_c) >= 0.002,
                  details=f"disp={math.dist(rest_c, sq_c):.5f}")
        sq_grip = ctx.part_element_world_aabb(lever, elem="grip_body")
        sq_grip_cy = (sq_grip[0][1] + sq_grip[1][1]) * 0.5
        ctx.check("squeeze swings the lever grip toward the body grip", rest_grip_cy - sq_grip_cy >= 0.005,
                  details=f"rest_cy={rest_grip_cy:.5f} sq_cy={sq_grip_cy:.5f}")
        ctx.expect_gap(grip_jaw_part, lever, axis="y", positive_elem="gripping_jaw_pad", negative_elem="stripping_jaw_pad",
                       min_gap=-0.001 if real_clamp else 0.0005, max_gap=0.003,
                       name="squeeze closes the wire slot without jaw contact")

    screw = ctx.part_element_world_aabb(grip_jaw_part, elem="tension_screw_head")
    ctx.check("brass tension screw sits proud on the head",
              screw is not None and screw[1][2] >= 0.018 and 0.04 <= (screw[0][0] + screw[1][0]) * 0.5 <= 0.08,
              details=f"screw aabb={screw}")

    ctx.expect_contact(slider, body, elem_a="slider_body", elem_b="slider_rail", name="slider rides on the rail")
    rest_s = ctx.part_world_aabb(slider)
    rest_sx = (rest_s[0][0] + rest_s[1][0]) * 0.5
    ctx.expect_overlap(slider, body, axes="x", elem_b="slider_rail", min_overlap=0.005, name="slider engages the rail at rest")
    with ctx.pose({slide: SLIDE_MAX}):
        out_s = ctx.part_world_aabb(slider)
        out_sx = (out_s[0][0] + out_s[1][0]) * 0.5
        ctx.check("wire stop slider slides toward the jaws", out_sx - rest_sx >= 0.006,
                  details=f"rest_sx={rest_sx:.5f} out_sx={out_sx:.5f}")
        ctx.expect_overlap(slider, body, axes="x", elem_b="slider_rail", min_overlap=0.005, name="slider stays engaged at full travel")
        ctx.expect_gap(grip_jaw_part, slider, axis="x", positive_elem="gripping_jaw", min_gap=0.0005,
                       name="slider stops short of the gripping jaw")
    ctx.expect_gap(slider, body, axis="x", negative_elem="stop_spring", min_gap=0.0, max_gap=0.003,
                   name="coil spring free end reaches the slider tail")

    if real_clamp:
        clamp = object_model.get_articulation("clamp_pinch")
        ctx.check("clamp_pinch is a REVOLUTE mimic DOF",
                  clamp.articulation_type == ArticulationType.REVOLUTE and clamp.motion_limits is not None
                  and clamp.motion_limits.upper > clamp.motion_limits.lower,
                  details=f"type={clamp.articulation_type}")
        rest_pad = ctx.part_element_world_aabb(grip_jaw_part, elem="gripping_jaw_pad")
        rest_pad_cy = (rest_pad[0][1] + rest_pad[1][1]) * 0.5
        with ctx.pose({pivot: r.squeeze_max}):
            sq_pad = ctx.part_element_world_aabb(grip_jaw_part, elem="gripping_jaw_pad")
            sq_pad_cy = (sq_pad[0][1] + sq_pad[1][1]) * 0.5
            ctx.check("squeeze drives clamp_pinch to close the gripping jaw", rest_pad_cy - sq_pad_cy >= 0.0005,
                      details=f"rest_cy={rest_pad_cy:.5f} sq_cy={sq_pad_cy:.5f}")


def run_wire_stripper_tests(object_model: ArticulatedObject, config: WireStripperConfig) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)

    if r.is_auto:
        _run_B_tests(object_model, r, ctx)
    else:
        _run_A_tests(object_model, r, ctx)

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.fail_if_joint_mating_has_gap()

    ctx.check("slot_choices recorded",
              tuple(object_model.meta.get("slot_choices", ())) == slot_choices_for_config(r),
              details=str(object_model.meta.get("slot_choices")))
    return ctx.report()


__all__ = (
    "WireStripperConfig",
    "ResolvedWireStripperConfig",
    "build_wire_stripper",
    "build_seeded_wire_stripper",
    "config_from_seed",
    "resolve_config",
    "run_wire_stripper_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
)
