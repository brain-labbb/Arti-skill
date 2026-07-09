"""Technology / Audio_Device — retro radio / boombox / transistor radio.

Modular procedural template (``__modular__ = True``, hand-dispatch parallel
children). A single ``body`` cabinet is the structural root; grille + control
+ handle + antenna modules parent to it. Structure is derived from the 9 5-star
sources declared in
``specs_modular_v1/Technology_Audio_Device.md``:

  ③ body_form      landscape_box (003/001/005) / oval_slab (002) /
                   tombstone_vertical (tombstone var of 003)
  A  grille        horizontal_ribbed (003/001) / perforated_mesh (005/002) /
                   vertical_bar (vertical_bar var)         [body.visual — Rule 1]
  B  speaker_layout single_center / dual_stereo (dual var of 002)
  C  controls      rotary_knob_bank (003 REVOLUTE / 001 CONTINUOUS) /
                   push_button_row (005 PRISMATIC ×N) /
                   transport_key_deck (002 PRISMATIC ×5 + 2 knob)  [≥1 non-fixed
                   joint every seed]
  D  handle        no_handle (003/002) / fixed_arched_bail (005 FIXED) /
                   folding_revolute_bail (001 REVOLUTE)
  E  antenna       none (003) / telescoping (001/005/002 REVOLUTE+PRISMATIC)
  multiplicity     button_count [2,8] (push_button_row) ; knob_count {2,3}

§A compliance: grille / dial / logo / wood grain are all ``body.visual`` (Rule
1). The single FIXED joint (fixed_arched_bail) carries a MatingContract on a
flat top-face weld (Rule 2); knob shafts / folding knuckle / antenna sleeve are
captured-pin/rotated-face geometry -> element-scoped ``allow_overlap`` grandfather
(Rule 2 exemption). Primitive types preserved from sources: cadquery filleted
bodies, PerforatedPanelGeometry / BezelGeometry grilles, KnobGeometry, swept
handle (Rule 3). Decoration is host-derived front-face geometry (Rule 4). Every
mechanism has a targeted ``ctx.pose`` check + harness_motion_qc sampled collision
(Rule 5).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from math import pi
from typing import Literal

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MatingContract,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

__modular__ = True

# --------------------------------------------------------------------------- #
# Enum domains
# --------------------------------------------------------------------------- #
BodyForm = Literal["landscape_box", "oval_slab", "tombstone_vertical"]
Grille = Literal["horizontal_ribbed", "perforated_mesh", "vertical_bar"]
SpeakerLayout = Literal["single_center", "dual_stereo"]
Controls = Literal["rotary_knob_bank", "push_button_row", "transport_key_deck"]
Handle = Literal["no_handle", "fixed_arched_bail", "folding_revolute_bail"]
Antenna = Literal["none", "telescoping"]
PaletteStyle = Literal[
    "cherry_wood", "tan_beige", "silver", "bronze_retro", "walnut", "matte_black"
]

BODY_FORMS: tuple[BodyForm, ...] = ("landscape_box", "oval_slab", "tombstone_vertical")
GRILLES: tuple[Grille, ...] = ("horizontal_ribbed", "perforated_mesh", "vertical_bar")
SPEAKER_LAYOUTS: tuple[SpeakerLayout, ...] = ("single_center", "dual_stereo")
CONTROLS: tuple[Controls, ...] = (
    "rotary_knob_bank",
    "push_button_row",
    "transport_key_deck",
)
HANDLES: tuple[Handle, ...] = (
    "no_handle",
    "fixed_arched_bail",
    "folding_revolute_bail",
)
ANTENNAS: tuple[Antenna, ...] = ("none", "telescoping")
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "cherry_wood",
    "tan_beige",
    "silver",
    "bronze_retro",
    "walnut",
    "matte_black",
)

BUTTON_COUNT_MIN, BUTTON_COUNT_MAX = 2, 8
KNOB_COUNT_DOMAIN: tuple[int, ...] = (2, 3)

# --------------------------------------------------------------------------- #
# Per-seed palette table (>=3 named tokens each; every visual pulls from here)
# --------------------------------------------------------------------------- #
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "cherry_wood": {
        "body_main": (0.62, 0.25, 0.12, 1.0),
        "body_trim": (0.34, 0.13, 0.06, 1.0),
        "grille_rim": (0.90, 0.84, 0.66, 1.0),
        "grille_mesh": (0.03, 0.03, 0.03, 1.0),
        "knob": (0.86, 0.86, 0.82, 1.0),
        "marker": (0.78, 0.06, 0.04, 1.0),
        "key": (0.89, 0.80, 0.58, 1.0),
        "handle": (0.06, 0.06, 0.06, 1.0),
        "antenna": (0.82, 0.83, 0.86, 1.0),
        "accent": (0.70, 0.60, 0.28, 1.0),
    },
    "tan_beige": {
        "body_main": (0.72, 0.52, 0.34, 1.0),
        "body_trim": (0.60, 0.43, 0.28, 1.0),
        "grille_rim": (0.80, 0.66, 0.44, 1.0),
        "grille_mesh": (0.02, 0.02, 0.02, 1.0),
        "knob": (0.86, 0.62, 0.25, 1.0),
        "marker": (0.30, 0.22, 0.14, 1.0),
        "key": (0.86, 0.79, 0.66, 1.0),
        "handle": (0.72, 0.52, 0.34, 1.0),
        "antenna": (0.86, 0.62, 0.25, 1.0),
        "accent": (0.86, 0.62, 0.25, 1.0),
    },
    "silver": {
        "body_main": (0.80, 0.82, 0.85, 1.0),
        "body_trim": (0.45, 0.47, 0.50, 1.0),
        "grille_rim": (0.62, 0.64, 0.68, 1.0),
        "grille_mesh": (0.08, 0.08, 0.09, 1.0),
        "knob": (0.18, 0.42, 0.88, 1.0),
        "marker": (0.90, 0.92, 0.95, 1.0),
        "key": (0.87, 0.87, 0.85, 1.0),
        "handle": (0.30, 0.32, 0.35, 1.0),
        "antenna": (0.74, 0.76, 0.80, 1.0),
        "accent": (0.18, 0.42, 0.88, 1.0),
    },
    "bronze_retro": {
        "body_main": (0.62, 0.40, 0.16, 1.0),
        "body_trim": (0.20, 0.12, 0.07, 1.0),
        "grille_rim": (0.30, 0.24, 0.16, 1.0),
        "grille_mesh": (0.05, 0.045, 0.04, 1.0),
        "knob": (0.09, 0.09, 0.10, 1.0),
        "marker": (0.80, 0.79, 0.82, 1.0),
        "key": (0.90, 0.86, 0.74, 1.0),
        "handle": (0.07, 0.07, 0.08, 1.0),
        "antenna": (0.78, 0.79, 0.82, 1.0),
        "accent": (0.80, 0.55, 0.22, 1.0),
    },
    "walnut": {
        "body_main": (0.36, 0.24, 0.14, 1.0),
        "body_trim": (0.22, 0.14, 0.08, 1.0),
        "grille_rim": (0.74, 0.66, 0.50, 1.0),
        "grille_mesh": (0.03, 0.028, 0.024, 1.0),
        "knob": (0.80, 0.79, 0.75, 1.0),
        "marker": (0.72, 0.12, 0.08, 1.0),
        "key": (0.82, 0.75, 0.60, 1.0),
        "handle": (0.10, 0.09, 0.07, 1.0),
        "antenna": (0.80, 0.80, 0.82, 1.0),
        "accent": (0.66, 0.52, 0.30, 1.0),
    },
    "matte_black": {
        "body_main": (0.11, 0.11, 0.12, 1.0),
        "body_trim": (0.18, 0.18, 0.20, 1.0),
        "grille_rim": (0.24, 0.24, 0.26, 1.0),
        "grille_mesh": (0.03, 0.03, 0.035, 1.0),
        "knob": (0.70, 0.71, 0.74, 1.0),
        "marker": (0.85, 0.18, 0.12, 1.0),
        "key": (0.55, 0.56, 0.58, 1.0),
        "handle": (0.14, 0.14, 0.16, 1.0),
        "antenna": (0.72, 0.73, 0.76, 1.0),
        "accent": (0.85, 0.55, 0.20, 1.0),
    },
}


# --------------------------------------------------------------------------- #
# Config dataclasses
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class AudioDeviceConfig:
    body_form: BodyForm | None = None
    grille: Grille | None = None
    speaker_layout: SpeakerLayout | None = None
    controls: Controls | None = None
    handle: Handle | None = None
    antenna: Antenna | None = None
    palette_style: PaletteStyle = "cherry_wood"
    button_count: int | None = None
    knob_count: int | None = None
    body_w: float = 0.30
    body_scale: float = 1.0
    knob_dia: float = 0.030
    button_travel: float = 0.004
    antenna_slide: float = 0.090
    grille_cover: float = 0.72
    name: str = "audio_device"


@dataclass(frozen=True)
class ResolvedAudioDeviceConfig:
    body_form: BodyForm
    grille: Grille
    speaker_layout: SpeakerLayout
    controls: Controls
    handle: Handle
    antenna: Antenna
    palette_style: PaletteStyle
    button_count: int
    knob_count: int
    # resolved dims
    body_w: float
    body_d: float
    body_h: float
    knob_dia: float
    button_travel: float
    antenna_slide: float
    grille_cover: float
    name: str


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _pick(value, choices):
    return value if value in choices else choices[0]


# --------------------------------------------------------------------------- #
# Seed sampling + resolution (procedural-first; seed 0 is NOT special)
# --------------------------------------------------------------------------- #
def config_from_seed(seed: int) -> AudioDeviceConfig:
    rng = random.Random(seed)
    body_form = rng.choices(BODY_FORMS, weights=[5, 3, 2], k=1)[0]
    grille = rng.choice(GRILLES)
    speaker_layout = rng.choices(SPEAKER_LAYOUTS, weights=[3, 2], k=1)[0]
    controls = rng.choice(CONTROLS)
    handle = rng.choices(HANDLES, weights=[3, 3, 3], k=1)[0]
    antenna = rng.choices(ANTENNAS, weights=[3, 4], k=1)[0]
    return AudioDeviceConfig(
        body_form=body_form,
        grille=grille,
        speaker_layout=speaker_layout,
        controls=controls,
        handle=handle,
        antenna=antenna,
        palette_style=rng.choice(PALETTE_STYLES),
        button_count=rng.choices([2, 3, 4, 5, 6, 7, 8], weights=[3, 5, 5, 5, 3, 2, 2], k=1)[0],
        knob_count=rng.choice(KNOB_COUNT_DOMAIN),
        body_w=round(rng.uniform(0.19, 0.36), 4),
        body_scale=round(rng.uniform(0.92, 1.12), 4),
        knob_dia=round(rng.uniform(0.016, 0.054), 4),
        button_travel=round(rng.uniform(0.0016, 0.005), 4),
        antenna_slide=round(rng.uniform(0.070, 0.106), 4),
        grille_cover=round(rng.uniform(0.60, 0.80), 4),
        name=f"seeded_audio_device_{seed}",
    )


# Per-form aspect equations (h,d as a fraction of width). Sources: 003/001/005
# landscape ~2:1 wide; 002 oval wide+deep; tombstone var tall.
_FORM_ASPECT: dict[str, tuple[float, float]] = {
    "landscape_box": (0.55, 0.45),      # (h/w, d/w)
    "oval_slab": (0.50, 0.68),
    "tombstone_vertical": (1.35, 0.55),
}


def resolve_config(config: AudioDeviceConfig | None = None) -> ResolvedAudioDeviceConfig:
    cfg = config or AudioDeviceConfig()
    body_form = _pick(cfg.body_form, BODY_FORMS)
    grille = _pick(cfg.grille, GRILLES)
    speaker_layout = _pick(cfg.speaker_layout, SPEAKER_LAYOUTS)
    controls = _pick(cfg.controls, CONTROLS)
    handle = _pick(cfg.handle, HANDLES)
    antenna = _pick(cfg.antenna, ANTENNAS)
    palette_style = _pick(cfg.palette_style, PALETTE_STYLES)

    # compatibility gate: a folding handle folds forward over the top face, so
    # it needs the top clear of controls -> only allowed with front-mounted
    # controls. push_button_row lives on the top face -> downgrade to a rigid
    # fixed bail (no motion into the button field).
    if handle == "folding_revolute_bail" and controls == "push_button_row":
        handle = "fixed_arched_bail"
    # tombstone/mantel radios are stationary — no carry bail (its wide saddles
    # would otherwise float off the arched top).
    if body_form == "tombstone_vertical":
        handle = "no_handle"

    button_count = int(
        _clamp(cfg.button_count if cfg.button_count is not None else 5,
               BUTTON_COUNT_MIN, BUTTON_COUNT_MAX)
    )
    knob_count = cfg.knob_count if cfg.knob_count in KNOB_COUNT_DOMAIN else 2
    if controls == "transport_key_deck":
        knob_count = 2  # deck carries exactly two rotary knobs

    # dims: width -> per-form aspect equation -> uniform body_scale
    s = _clamp(cfg.body_scale, 0.92, 1.12)
    bw = _clamp(cfg.body_w, 0.19, 0.36)
    h_frac, d_frac = _FORM_ASPECT[body_form]
    body_w = round(bw * s, 5)
    body_h = round(bw * h_frac * s, 5)
    body_d = round(bw * d_frac * s, 5)

    return ResolvedAudioDeviceConfig(
        body_form=body_form,
        grille=grille,
        speaker_layout=speaker_layout,
        controls=controls,
        handle=handle,
        antenna=antenna,
        palette_style=palette_style,
        button_count=button_count,
        knob_count=knob_count,
        body_w=body_w,
        body_d=body_d,
        body_h=body_h,
        knob_dia=_clamp(cfg.knob_dia, 0.016, 0.054),
        button_travel=_clamp(cfg.button_travel, 0.0016, 0.005),
        antenna_slide=_clamp(cfg.antenna_slide, 0.070, 0.106),
        grille_cover=_clamp(cfg.grille_cover, 0.60, 0.80),
        name=cfg.name or "audio_device",
    )


def slot_choices_for_config(
    config: AudioDeviceConfig | ResolvedAudioDeviceConfig,
) -> tuple[tuple[str, str], ...]:
    r = config if isinstance(config, ResolvedAudioDeviceConfig) else resolve_config(config)
    choices: list[tuple[str, str]] = [
        ("body_form", r.body_form),
        ("grille", r.grille),
        ("speaker_layout", r.speaker_layout),
        ("controls", r.controls),
        ("handle", r.handle),
        ("antenna", r.antenna),
    ]
    if r.controls == "push_button_row":
        choices.append(("button_count", f"n_{r.button_count}"))
    if r.controls in ("rotary_knob_bank", "transport_key_deck"):
        choices.append(("knob_count", f"k_{r.knob_count}"))
    return tuple(choices)


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


# --------------------------------------------------------------------------- #
# Front-face / top-face layout (single-sourced geometric quantities)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class _Layout:
    front_y: float
    top_z: float
    lower_strip: bool     # controls sit on the lower front strip
    strip_cz: float       # z of the front control strip (knobs / transport keys)
    grille_cx: float
    grille_cz: float
    grille_w: float
    grille_h: float
    spk_offset: float     # |x| of each driver for dual_stereo


def _top_z_at(r: ResolvedAudioDeviceConfig, x: float) -> float:
    """World top-face z at a given x (flat for box/oval, arched for tombstone)."""
    if r.body_form != "tombstone_vertical":
        return r.body_h
    arch_r = r.body_w / 2.0
    straight_h = max(0.02, r.body_h - arch_r)
    return straight_h + math.sqrt(max(0.0, arch_r * arch_r - x * x))


def _layout(r: ResolvedAudioDeviceConfig) -> _Layout:
    bw, bd, bh = r.body_w, r.body_d, r.body_h
    front_y = -bd / 2.0
    tomb = r.body_form == "tombstone_vertical"
    lower_strip = r.controls in ("rotary_knob_bank", "transport_key_deck")

    strip_cz = (0.15 if tomb else 0.20) * bh
    if tomb:
        g_bot = (0.32 if lower_strip else 0.12) * bh
        g_top = 0.58 * bh
    else:
        g_bot = (0.44 if lower_strip else 0.12) * bh
        g_top = 0.93 * bh
    grille_cz = 0.5 * (g_bot + g_top)
    grille_h = g_top - g_bot
    grille_w = r.grille_cover * bw
    spk_offset = 0.5 * grille_w - 0.24 * grille_w  # centers of two half-grilles
    return _Layout(
        front_y=front_y,
        top_z=bh,
        lower_strip=lower_strip,
        strip_cz=strip_cz,
        grille_cx=0.0,
        grille_cz=grille_cz,
        grille_w=grille_w,
        grille_h=grille_h,
        spk_offset=spk_offset,
    )


# --------------------------------------------------------------------------- #
# Body form meshes (Rule 3: keep the cadquery filleted / arched solids)
# --------------------------------------------------------------------------- #
def _landscape_solid(bw: float, bd: float, bh: float) -> cq.Workplane:
    r = min(0.026, 0.12 * min(bw, bh))
    body = (
        cq.Workplane("XY")
        .box(bw, bd, bh, centered=(True, True, False))
        .edges("|Z")
        .fillet(r)
    )
    try:
        body = body.edges(">Z").fillet(r * 0.35)
    except Exception:
        pass
    return body


def _oval_solid(bw: float, bd: float, bh: float) -> cq.Workplane:
    corner = min(0.34 * min(bw, bd), 0.44 * bd)
    body = (
        cq.Workplane("XY")
        .box(bw, bd, bh, centered=(True, True, False))
        .edges("|Z")
        .fillet(corner)
    )
    try:
        body = body.edges(">Z").fillet(min(0.013, 0.10 * bh))
        body = body.edges("<Z").fillet(min(0.006, 0.05 * bh))
    except Exception:
        pass
    return body


def _tombstone_solid(bw: float, bd: float, bh: float) -> cq.Workplane:
    arch_r = bw / 2.0
    straight_h = max(0.02, bh - arch_r)
    profile = (
        cq.Workplane("XZ")
        .moveTo(-bw / 2.0, 0.0)
        .lineTo(bw / 2.0, 0.0)
        .lineTo(bw / 2.0, straight_h)
        .threePointArc((0.0, bh), (-bw / 2.0, straight_h))
        .close()
    )
    body = profile.extrude(bd / 2.0, both=True)
    try:
        body = body.edges("|Y").fillet(min(0.014, 0.06 * bw))
        body = body.edges("<Z").fillet(0.006)
    except Exception:
        pass
    return body


def _body_solid(r: ResolvedAudioDeviceConfig) -> cq.Workplane:
    if r.body_form == "oval_slab":
        return _oval_solid(r.body_w, r.body_d, r.body_h)
    if r.body_form == "tombstone_vertical":
        return _tombstone_solid(r.body_w, r.body_d, r.body_h)
    return _landscape_solid(r.body_w, r.body_d, r.body_h)


# --------------------------------------------------------------------------- #
# Body part + host-conformal decoration + grille (all body.visual — Rule 1/4)
# --------------------------------------------------------------------------- #
def _emit_body(model, r, mats, lay: _Layout):
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(r), "body_shell", tolerance=0.0011, angular_tolerance=0.2),
        material=mats["body_main"],
        name="body_shell",
    )

    front_y = lay.front_y
    bw, bd, bh = r.body_w, r.body_d, r.body_h

    # ④ host-conformal front trim band framing the front face (derived from
    # the final front plane + body dims).
    body.visual(
        Box((bw * 0.94, 0.006, bh * 0.90)),
        origin=Origin(xyz=(0.0, front_y + 0.002, bh * 0.5)),
        material=mats["body_trim"],
        name="front_trim",
    )

    # ④ wood/finish grain strips on the top deck (host-conformal: each strip
    # seats on the real top height at its x — arched for tombstone).
    n_grain = 6
    for i in range(n_grain):
        gx = (-0.5 + (i + 0.5) / n_grain) * bw * 0.82
        body.visual(
            Box((0.004, bd * 0.7, 0.0012)),
            origin=Origin(xyz=(gx, 0.006 * ((i % 3) - 1.0), _top_z_at(r, gx) + 0.0002)),
            material=mats["body_trim"],
            name=f"top_grain_{i}",
        )

    # ④ printed dial-scale strip below the grille (front visual, on the strip).
    body.visual(
        Box((bw * 0.5, 0.004, bh * 0.05)),
        origin=Origin(xyz=(0.0, front_y - 0.001, lay.grille_cz - lay.grille_h * 0.5 - bh * 0.05)),
        material=mats["accent"],
        name="dial_scale",
    )

    # oval boombox feet (source 002 foot_{i})
    if r.body_form == "oval_slab":
        for i, (fx, fy) in enumerate(
            ((-0.34 * bw, -0.30 * bd), (0.34 * bw, -0.30 * bd),
             (-0.34 * bw, 0.30 * bd), (0.34 * bw, 0.30 * bd))
        ):
            body.visual(
                Cylinder(radius=0.013, length=0.006),
                origin=Origin(xyz=(fx, fy, 0.001)),
                material=mats["handle"],
                name=f"foot_{i}",
            )

    # A: speaker grille (+ B: single / dual) — pure body visuals.
    if r.speaker_layout == "dual_stereo":
        _emit_grille(body, r, mats, lay, cx=-lay.spk_offset, w=lay.grille_w * 0.46, suffix="_0")
        _emit_grille(body, r, mats, lay, cx=lay.spk_offset, w=lay.grille_w * 0.46, suffix="_1")
    else:
        _emit_grille(body, r, mats, lay, cx=lay.grille_cx, w=lay.grille_w, suffix="")

    return body


def _emit_grille(body, r, mats, lay: _Layout, *, cx: float, w: float, suffix: str) -> None:
    """Slot A grille construction as body visuals just behind the front plane."""
    front_y = lay.front_y
    gh = lay.grille_h
    gz = lay.grille_cz

    # dark speaker backing (shared by all constructions)
    body.visual(
        Box((w * 0.98, 0.002, gh * 0.96)),
        origin=Origin(xyz=(cx, front_y + 0.0018, gz)),
        material=mats["grille_mesh"],
        name=f"speaker_backing{suffix}",
    )

    if r.grille == "perforated_mesh":
        bezel = BezelGeometry(
            (w * 0.90, gh * 0.90),
            (w, gh),
            0.006,
            opening_shape="rounded_rect",
            outer_shape="rounded_rect",
            opening_corner_radius=min(0.012, 0.10 * gh),
            outer_corner_radius=min(0.018, 0.14 * gh),
        )
        body.visual(
            mesh_from_geometry(bezel, f"grille_bezel{suffix}"),
            origin=Origin(xyz=(cx, front_y - 0.001, gz), rpy=(pi / 2.0, 0.0, 0.0)),
            material=mats["grille_rim"],
            name=f"grille_bezel{suffix}",
        )
        panel = PerforatedPanelGeometry(
            (w * 0.86, gh * 0.86),
            0.004,
            hole_diameter=0.005,
            pitch=(0.0105, 0.0095),
            frame=0.006,
            corner_radius=0.007,
            stagger=True,
        )
        body.visual(
            mesh_from_geometry(panel, f"speaker_grille{suffix}"),
            origin=Origin(xyz=(cx, front_y + 0.0005, gz), rpy=(pi / 2.0, 0.0, 0.0)),
            material=mats["grille_mesh"],
            name=f"speaker_grille{suffix}",
        )
        return

    # ribbed / bar: rails + a loop of thin slats (Box loop, cheap — source 003).
    rail_y = front_y - 0.001
    body.visual(
        Box((w, 0.010, 0.008)),
        origin=Origin(xyz=(cx, rail_y, gz + gh / 2.0 - 0.004)),
        material=mats["grille_rim"],
        name=f"grille_top_rail{suffix}",
    )
    body.visual(
        Box((w, 0.010, 0.008)),
        origin=Origin(xyz=(cx, rail_y, gz - gh / 2.0 + 0.004)),
        material=mats["grille_rim"],
        name=f"grille_bottom_rail{suffix}",
    )
    for side, sx in enumerate((cx - w / 2.0 + 0.005, cx + w / 2.0 - 0.005)):
        body.visual(
            Box((0.010, 0.010, gh)),
            origin=Origin(xyz=(sx, rail_y, gz)),
            material=mats["grille_rim"],
            name=f"grille_side_rail_{side}{suffix}",
        )

    if r.grille == "vertical_bar":
        span = w - 0.024
        n = max(6, min(20, int(span / 0.012)))
        for i in range(n):
            x = cx - span / 2.0 + span * (i / (n - 1)) if n > 1 else cx
            body.visual(
                Box((0.0035, 0.010, gh - 0.014)),
                origin=Origin(xyz=(x, front_y - 0.0015, gz)),
                material=mats["grille_rim"],
                name=f"grille_bar_{i}{suffix}",
            )
    else:  # horizontal_ribbed
        span = gh - 0.014
        n = max(6, min(18, int(span / 0.0085)))
        for i in range(n):
            z = gz - span / 2.0 + span * (i / (n - 1)) if n > 1 else gz
            body.visual(
                Box((w - 0.014, 0.010, 0.0038)),
                origin=Origin(xyz=(cx, front_y - 0.0015, z)),
                material=mats["grille_rim"],
                name=f"grille_rib_{i}{suffix}",
            )


# --------------------------------------------------------------------------- #
# Slot C: controls (every branch emits >=1 non-fixed joint)
# --------------------------------------------------------------------------- #
def _knob_mesh(dia: float):
    # domed + light fluting, NO skirt — the skirt boolean is ~8s; domed ~1s.
    h = max(0.010, dia * 0.5)
    return mesh_from_geometry(
        KnobGeometry(
            dia,
            h,
            body_style="domed",
            grip=KnobGrip(style="fluted", count=14, depth=0.0011),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "knob_cap",
    )


def _emit_front_knob(model, r, body, mats, name, x, z, *, continuous: bool, dia: float, mesh):
    """A rotary knob on the front face spinning about the front normal (-Y).

    Cap local +Z -> world -Y (out the front) via rpy=(+pi/2,0,0). An off-axis
    pointer tab makes the axisymmetric cap fail a no-rotation AABB check. A
    hidden shaft goes inward (+Y) into the housing -> element-scoped allow_overlap.
    """
    kh = max(0.010, dia * 0.5)
    knob = model.part(name)
    knob.visual(mesh, origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
                material=mats["knob"], name=f"{name}_cap")
    # hidden shaft bridging the knob back into the housing (intentional embed)
    knob.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=mats["accent"],
        name=f"{name}_shaft",
    )
    # off-axis pointer merged onto the cap tip (proves rotation about -Y)
    knob.visual(
        Box((0.0026, 0.0016, 0.0026)),
        origin=Origin(xyz=(0.0, -kh - 0.0004, dia * 0.30)),
        material=mats["marker"],
        name=f"{name}_pointer",
    )
    jtype = ArticulationType.CONTINUOUS if continuous else ArticulationType.REVOLUTE
    limits = (
        MotionLimits(effort=0.5, velocity=6.0)
        if continuous
        else MotionLimits(effort=0.6, velocity=3.5, lower=-pi, upper=pi)
    )
    model.articulation(
        f"cab_to_{name}",
        jtype,
        parent=body,
        child=knob,
        origin=Origin(xyz=(x, r.body_d / -2.0, z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )


def _emit_controls(model, r, body, mats, lay: _Layout) -> list[str]:
    bw, bd, bh = r.body_w, r.body_d, r.body_h
    strip_cz = lay.strip_cz

    # knob diameter capped to fit the strip (small bodies / large sampled dia).
    kd = min(r.knob_dia, 0.18 * bh, 0.30 * bw)

    if r.controls == "rotary_knob_bank":
        names = []
        mesh = _knob_mesh(kd)  # one shared mesh for every knob in the bank
        if r.knob_count == 3:
            xs = (-0.30 * bw, 0.0, 0.30 * bw)
        else:
            xs = (-0.28 * bw, 0.28 * bw)
        for i, x in enumerate(xs):
            nm = f"knob_{i}"
            _emit_front_knob(model, r, body, mats, nm, x, strip_cz, continuous=False, dia=kd, mesh=mesh)
            names.append(nm)
        return names

    if r.controls == "transport_key_deck":
        # two CONTINUOUS knobs flanking the transport-key row on the same strip
        names = []
        mesh = _knob_mesh(kd)
        for i, x in enumerate((-0.36 * bw, 0.36 * bw)):
            nm = f"knob_{i}"
            _emit_front_knob(model, r, body, mats, nm, x, strip_cz,
                             continuous=True, dia=kd, mesh=mesh)
            names.append(nm)
        # five transport keys pressing inward (+Y), centered between the knobs
        key_mesh = mesh_from_cadquery(
            cq.Workplane("XY").box(0.016, 0.012, 0.010).edges("|Y").fillet(0.002),
            "key_cap", tolerance=0.0006,
        )
        span = min(0.40 * bw, 0.11)
        for i in range(5):
            x = -span / 2.0 + span * (i / 4.0)
            nm = f"transport_key_{i}"
            key = model.part(nm)
            key.visual(key_mesh, origin=Origin(xyz=(0.0, -0.004, 0.0)),
                       material=mats["key"], name="key_cap")
            model.articulation(
                f"body_to_{nm}",
                ArticulationType.PRISMATIC,
                parent=body,
                child=key,
                origin=Origin(xyz=(x, lay.front_y, strip_cz)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=0.06, lower=0.0, upper=r.button_travel),
            )
            names.append(nm)
        return names

    # push_button_row — a row of N top-face buttons pressing down (-Z)
    names = []
    btn_mesh = mesh_from_cadquery(
        cq.Workplane("XY").box(0.024, 0.016, 0.008).edges("|Z").fillet(0.0025),
        "button_cap", tolerance=0.0006,
    )
    n = r.button_count
    span = min(0.48 * bw, 0.026 * n)
    for idx in range(n):
        x = -span / 2.0 + span * (idx / (n - 1)) if n > 1 else 0.0
        nm = f"button_{idx}"
        btn = model.part(nm)
        btn.visual(btn_mesh, origin=Origin(xyz=(0.0, 0.0, 0.004)),
                   material=mats["key"], name="button_cap")
        model.articulation(
            f"body_to_{nm}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=btn,
            # seat on the real top height at this x (arched for tombstone),
            # embedded ~1.5mm so the faceted apex mesh still registers contact.
            origin=Origin(xyz=(x, -0.12 * bd, _top_z_at(r, x) - 0.0015)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.06, lower=0.0, upper=r.button_travel),
        )
        names.append(nm)
    return names


# --------------------------------------------------------------------------- #
# Slot D: carry handle
# --------------------------------------------------------------------------- #
def _handle_arch_geom(bw: float, bh: float):
    half = 0.40 * bw
    top_z = bh + 0.04 + 0.10 * bw
    path = [
        (-half, 0.0, bh + 0.006),
        (-half + 0.30 * half, 0.0, top_z * 0.86),
        (0.0, 0.0, top_z),
        (half - 0.30 * half, 0.0, top_z * 0.86),
        (half, 0.0, bh + 0.006),
    ]
    return sweep_profile_along_spline(
        path,
        profile=rounded_rect_profile(0.014, 0.010, radius=0.003),
        samples_per_segment=10,
        spline="catmull_rom",
        cap_profile=True,
    )


def _emit_handle(model, r, body, mats, lay: _Layout) -> None:
    if r.handle == "no_handle":
        return
    bw, bd, bh = r.body_w, r.body_d, r.body_h
    handle = model.part("carry_handle")
    half = 0.40 * bw

    # FIXED bail welds at the left saddle; folding bail pivots at top center.
    # The child part frame is placed at the joint origin, so every handle visual
    # is authored in world coords then rebased by -anchor (local = world-anchor).
    anchor = (-half, 0.0, bh) if r.handle == "fixed_arched_bail" else (0.0, 0.0, bh)

    def _sub(p):
        return (p[0] - anchor[0], p[1] - anchor[1], p[2] - anchor[2])

    for k, x in enumerate((-half, half)):
        handle.visual(
            Box((0.026, 0.024, 0.030)),
            origin=Origin(xyz=_sub((x, 0.0, bh + 0.014))),
            material=mats["handle"],
            name=f"handle_saddle_{k}",
        )
    # arch geometry carries absolute world coords -> visual origin = -anchor.
    handle.visual(
        mesh_from_geometry(_handle_arch_geom(bw, bh), "handle_arch"),
        origin=Origin(xyz=(-anchor[0], -anchor[1], -anchor[2])),
        material=mats["handle"],
        name="handle_arch",
    )

    if r.handle == "fixed_arched_bail":
        model.articulation(
            "body_to_handle",
            ArticulationType.FIXED,
            parent=body,
            child=handle,
            origin=Origin(xyz=anchor),
            mating=MatingContract(
                parent_face_geometry="body_shell",
                parent_face_side="positive_z",
                child_face_geometry="handle_saddle_0",
                child_face_side="negative_z",
                contact_tol=0.0025,
            ),
        )
    else:  # folding_revolute_bail — pivot about X on the flat top, folds forward
        # The pivot axis lies on the (flat) top face, so it touches body hardware
        # directly — no separate boss needed (avoids an arch/boss overlap island).
        model.articulation(
            "body_to_handle",
            ArticulationType.REVOLUTE,
            parent=body,
            child=handle,
            origin=Origin(xyz=anchor),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=pi / 2.0),
        )


# --------------------------------------------------------------------------- #
# Slot E: antenna (telescoping: base REVOLUTE swivel + rod PRISMATIC)
# --------------------------------------------------------------------------- #
def _emit_antenna(model, r, body, mats, lay: _Layout) -> None:
    if r.antenna == "none":
        return
    bw, bd, bh = r.body_w, r.body_d, r.body_h
    # tombstone top is arched -> mount near the high center so the boss seats on
    # real geometry; box/oval tops are flat so a rear-right corner is fine.
    ax = 0.22 * bw if r.body_form == "tombstone_vertical" else 0.34 * bw
    ay = 0.30 * bd
    mount_z = _top_z_at(r, ax)
    # mounting boss on the top deck (body visual), seated onto the top surface
    body.visual(
        Cylinder(radius=0.0085, length=0.014),
        origin=Origin(xyz=(ax, ay, mount_z + 0.003)),
        material=mats["handle"],
        name="antenna_boss",
    )
    boss_top = mount_z + 0.010

    base = model.part("antenna_base")
    base.visual(
        Cylinder(radius=0.0068, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=mats["antenna"],
        name="antenna_knuckle",
    )
    sleeve_len = 0.090
    base.visual(
        Cylinder(radius=0.0048, length=sleeve_len),
        origin=Origin(xyz=(0.0, 0.0, 0.012 + sleeve_len / 2.0)),
        material=mats["antenna"],
        name="antenna_mast",
    )
    model.articulation(
        "antenna_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=base,
        # seat the knuckle ~4mm into the boss so isolated-parts (1µm tol) holds.
        origin=Origin(xyz=(ax, ay, boss_top - 0.005)),
        axis=(1.0, 0.0, 0.0),
        # rakes REARWARD (+Y) only, away from the front controls, the top push
        # buttons and the carry handle (which folds forward, -Y).
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-math.radians(85.0), upper=0.0),
    )

    rod = model.part("antenna_rod")
    rod_len = r.antenna_slide + 0.030
    rod.visual(
        Cylinder(radius=0.0024, length=rod_len),
        origin=Origin(xyz=(0.0, 0.0, rod_len / 2.0)),
        material=mats["antenna"],
        name="antenna_rod",
    )
    rod.visual(
        Sphere(radius=0.0044),
        origin=Origin(xyz=(0.0, 0.0, rod_len)),
        material=mats["antenna"],
        name="antenna_tip",
    )
    # rest pose extended; retracting slides the rod down into the sleeve.
    model.articulation(
        "antenna_extend",
        ArticulationType.PRISMATIC,
        parent=base,
        child=rod,
        origin=Origin(xyz=(0.0, 0.0, 0.012 + sleeve_len - 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.2, lower=-r.antenna_slide, upper=0.0),
    )


# --------------------------------------------------------------------------- #
# Top-level builders
# --------------------------------------------------------------------------- #
def build_audio_device(
    config: AudioDeviceConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=r.name, assets=assets)

    palette = PALETTES[r.palette_style]
    mats = {
        key: model.material(f"ad_{key}_{r.palette_style}", rgba=rgba)
        for key, rgba in palette.items()
    }

    lay = _layout(r)
    body = _emit_body(model, r, mats, lay)
    _emit_controls(model, r, body, mats, lay)  # Slot C — guarantees non-fixed joint
    _emit_handle(model, r, body, mats, lay)     # Slot D
    _emit_antenna(model, r, body, mats, lay)    # Slot E

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_audio_device(
    seed: int,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    return build_audio_device(config_from_seed(seed), assets=assets)


# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #
def _control_part_names(r: ResolvedAudioDeviceConfig) -> list[str]:
    if r.controls == "rotary_knob_bank":
        return [f"knob_{i}" for i in range(r.knob_count)]
    if r.controls == "transport_key_deck":
        return [f"knob_{i}" for i in range(2)] + [f"transport_key_{i}" for i in range(5)]
    return [f"button_{i}" for i in range(r.button_count)]


def run_audio_device_tests(
    object_model: ArticulatedObject,
    config: AudioDeviceConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    # ---- intentional embeddings (element-scoped, captured-pin / seated caps) --
    if r.controls == "rotary_knob_bank":
        for i in range(r.knob_count):
            k = object_model.get_part(f"knob_{i}")
            ctx.allow_overlap(k, body, elem_a=f"knob_{i}_cap", elem_b="body_shell",
                              reason="knob skirt is seated against the front face")
            ctx.allow_overlap(k, body, elem_a=f"knob_{i}_shaft", elem_b="body_shell",
                              reason="knob shaft passes into the housing")
            ctx.allow_overlap(k, body, elem_a=f"knob_{i}_cap", elem_b="front_trim",
                              reason="knob skirt seats against the front trim band")
    elif r.controls == "transport_key_deck":
        for i in range(2):
            k = object_model.get_part(f"knob_{i}")
            ctx.allow_overlap(k, body, elem_a=f"knob_{i}_cap", elem_b="body_shell",
                              reason="knob skirt is seated against the front face")
            ctx.allow_overlap(k, body, elem_a=f"knob_{i}_shaft", elem_b="body_shell",
                              reason="knob shaft passes into the housing")
            ctx.allow_overlap(k, body, elem_a=f"knob_{i}_cap", elem_b="front_trim",
                              reason="knob skirt seats against the front trim band")
        for i in range(5):
            key = object_model.get_part(f"transport_key_{i}")
            ctx.allow_overlap(key, body, elem_a="key_cap", elem_b="body_shell",
                              reason="transport key is seated into the front so it can press inward")
            ctx.allow_overlap(key, body, elem_a="key_cap", elem_b="front_trim",
                              reason="transport key passes through the front trim band")
    else:
        for i in range(r.button_count):
            b = object_model.get_part(f"button_{i}")
            ctx.allow_overlap(b, body, elem_a="button_cap", elem_b="body_shell",
                              reason="button cap seats into the top deck so it can press down")

    if r.handle == "fixed_arched_bail":
        h = object_model.get_part("carry_handle")
        for k in (0, 1):
            ctx.allow_overlap(h, body, elem_a=f"handle_saddle_{k}", elem_b="body_shell",
                              reason="handle saddle seats onto the top face")
    elif r.handle == "folding_revolute_bail":
        h = object_model.get_part("carry_handle")
        for k in (0, 1):
            ctx.allow_overlap(h, body, elem_a=f"handle_saddle_{k}", elem_b="body_shell",
                              reason="folding handle feet seat onto the top face / fold flat over it")
        ctx.allow_overlap(h, body, elem_a="handle_arch", elem_b="body_shell",
                          reason="folded handle arch rests over the top / front of the housing")

    if r.antenna == "telescoping":
        base = object_model.get_part("antenna_base")
        rod = object_model.get_part("antenna_rod")
        ctx.allow_overlap(base, body, elem_a="antenna_knuckle", elem_b="antenna_boss",
                          reason="antenna swivel knuckle seats on the mounting boss")
        ctx.allow_overlap(base, body, elem_a="antenna_knuckle", elem_b="body_shell",
                          reason="captured swivel knuckle seats into the top deck and grazes it while raking")
        ctx.allow_overlap(rod, base, elem_a="antenna_rod", elem_b="antenna_mast",
                          reason="thin rod telescopes inside the hollow mast sleeve")

    # ---- baseline gates (element allowances registered above are honored) ----
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.fail_if_joint_mating_has_gap()

    # ---- identity: single grounded root, wider-or-taller cabinet ----
    roots = object_model.root_parts()
    ctx.check(
        "single_body_root",
        len(roots) == 1 and roots[0].name == "body",
        details=f"roots={[p.name for p in roots]}",
    )
    baabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        baabb is not None and abs(baabb[0][2]) < 0.01,
        details=f"zmin={None if baabb is None else baabb[0][2]:.4f}",
    )

    # ---- at least one non-fixed joint every seed (control mechanism) ----
    control_names = _control_part_names(r)
    ctx.check(
        "controls_emit_movable_parts",
        all(object_model.get_part(nm) is not None for nm in control_names) and len(control_names) >= 1,
        details=f"controls={r.controls} parts={control_names}",
    )

    # ---- rotary knob spins: off-axis pointer sweeps ----
    if r.controls in ("rotary_knob_bank", "transport_key_deck"):
        knob = object_model.get_part("knob_0")
        j = object_model.get_articulation("cab_to_knob_0")
        continuous = j.articulation_type == ArticulationType.CONTINUOUS
        ctx.check(
            "knob_0_rotates_about_front_normal",
            j.articulation_type in (ArticulationType.REVOLUTE, ArticulationType.CONTINUOUS)
            and tuple(round(c, 3) for c in j.axis) == (0.0, -1.0, 0.0),
            details=f"type={j.articulation_type} axis={tuple(j.axis)}",
        )
        p0 = ctx.part_element_world_aabb(knob, elem="knob_0_pointer")
        if p0 is not None:
            z0 = 0.5 * (p0[0][2] + p0[1][2])
            with ctx.pose({j: pi}):
                p1 = ctx.part_element_world_aabb(knob, elem="knob_0_pointer")
            z1 = 0.5 * (p1[0][2] + p1[1][2])
            ctx.check(
                "knob_0_pointer_sweeps_off_axis",
                abs(z1 - z0) > 0.006,
                details=f"pointer z q=0:{z0:.5f} q=pi:{z1:.5f} continuous={continuous}",
            )

    # ---- push / transport keys press along their axis ----
    if r.controls == "push_button_row":
        b0 = object_model.get_part("button_0")
        jb = object_model.get_articulation("body_to_button_0")
        ctx.check(
            "button_0_prismatic_down",
            jb.articulation_type == ArticulationType.PRISMATIC
            and tuple(round(c, 3) for c in jb.axis) == (0.0, 0.0, -1.0),
            details=f"axis={tuple(jb.axis)}",
        )
        rest = ctx.part_world_position(b0)
        with ctx.pose({jb: r.button_travel}):
            pressed = ctx.part_world_position(b0)
        ctx.check(
            "button_0_travels_down",
            rest is not None and pressed is not None and pressed[2] < rest[2] - 0.6 * r.button_travel,
            details=f"rest={rest} pressed={pressed}",
        )
    elif r.controls == "transport_key_deck":
        k0 = object_model.get_part("transport_key_0")
        jk = object_model.get_articulation("body_to_transport_key_0")
        ctx.check(
            "transport_key_0_prismatic_inward",
            jk.articulation_type == ArticulationType.PRISMATIC
            and tuple(round(c, 3) for c in jk.axis) == (0.0, 1.0, 0.0),
            details=f"axis={tuple(jk.axis)}",
        )
        rest = ctx.part_world_position(k0)
        with ctx.pose({jk: r.button_travel}):
            pressed = ctx.part_world_position(k0)
        ctx.check(
            "transport_key_0_presses_inward",
            rest is not None and pressed is not None and pressed[1] > rest[1] + 0.6 * r.button_travel,
            details=f"rest={rest} pressed={pressed}",
        )

    # ---- carry handle ----
    if r.handle == "fixed_arched_bail":
        jh = object_model.get_articulation("body_to_handle")
        ctx.check("handle_fixed", jh.articulation_type == ArticulationType.FIXED,
                  details=f"type={jh.articulation_type}")
    elif r.handle == "folding_revolute_bail":
        jh = object_model.get_articulation("body_to_handle")
        ctx.check(
            "handle_revolute_x_0_to_90",
            jh.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(c, 3) for c in jh.axis) == (1.0, 0.0, 0.0),
            details=f"type={jh.articulation_type} axis={tuple(jh.axis)}",
        )
        h = object_model.get_part("carry_handle")
        c0 = ctx.part_world_aabb(h)
        with ctx.pose({jh: pi / 2.0}):
            c1 = ctx.part_world_aabb(h)
        ctx.check(
            "folding_handle_drops_when_folded",
            c0 is not None and c1 is not None and c1[1][2] < c0[1][2] - 0.02,
            details=f"rest_top={c0[1][2]:.4f} folded_top={c1[1][2]:.4f}",
        )

    # ---- antenna telescopes + swivels ----
    if r.antenna == "telescoping":
        rod = object_model.get_part("antenna_rod")
        base = object_model.get_part("antenna_base")
        je = object_model.get_articulation("antenna_extend")
        js = object_model.get_articulation("antenna_swivel")
        ctx.check(
            "antenna_extend_prismatic",
            je.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={je.articulation_type}",
        )
        rest_tip = ctx.part_world_aabb(rod)[1][2]
        with ctx.pose({je: -r.antenna_slide}):
            coll_tip = ctx.part_world_aabb(rod)[1][2]
        ctx.check(
            "antenna_tip_lowers_when_collapsed",
            rest_tip - coll_tip > 0.5 * r.antenna_slide,
            details=f"ext_tip={rest_tip:.4f} coll_tip={coll_tip:.4f}",
        )
        rest_pos = ctx.part_world_position(rod)
        with ctx.pose({js: -math.radians(75.0)}):
            sw_pos = ctx.part_world_position(rod)
        ctx.check(
            "antenna_swivels_at_base",
            rest_pos is not None and sw_pos is not None
            and abs(sw_pos[1] - rest_pos[1]) > 0.02,
            details=f"rest={tuple(round(v,3) for v in rest_pos)} swiveled={tuple(round(v,3) for v in sw_pos)}",
        )

    # ---- slot_choices recorded ----
    ctx.check(
        "slot_choices_recorded",
        tuple(object_model.meta.get("slot_choices", ())) == slot_choices_for_config(r),
        details=str(object_model.meta.get("slot_choices")),
    )

    return ctx.report()


__all__ = (
    "AudioDeviceConfig",
    "ResolvedAudioDeviceConfig",
    "build_audio_device",
    "build_seeded_audio_device",
    "config_from_seed",
    "resolve_config",
    "run_audio_device_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
)
