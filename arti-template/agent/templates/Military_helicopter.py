from __future__ import annotations

# Procedural template: CH-47-style TANDEM-ROTOR heavy-lift helicopter.
#
# Two main lift rotors in tandem (low front pylon + tall rear pylon, opposite
# spin), no tail rotor. Twin engine nacelles, side fuel sponsons carrying a
# four-wheel undercarriage, and an upswept tail with a rear cargo ramp.
#
# Axes (all sourced from hand-built, QC-passed workbench variants):
#   - airframe body:   box / bellypod / humpback / stretch        (Slot A, 4)
#   - landing gear:    fixed_strut / oleo_telescoping              (Slot C, 2)
#   - doors:           none / cockpit_door / bulkhead_door / both  (Slot D, 4)
#   - front/rear rotor blade counts: 2..8 each (independent multiplicity)
#   - seats per side / exhaust nozzle count: multiplicity
#   - pointed vs blunt nose: random toggle
#
# Geometry conventions (real meters, Z-up): nose = +X, tail = -X, width along Y;
# wheels touch the ground at z = 0. The whole airframe is lifted by Z_LIFT.

import math
import random
from dataclasses import dataclass, field
from typing import Literal

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    LoftSection,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    section_loft,
)

__modular__ = True

# ---------------------------------------------------------------------------
# Module enums
# ---------------------------------------------------------------------------
AirframeModule = Literal["box", "bellypod", "humpback", "stretch"]
GearModule = Literal["fixed_strut", "oleo_telescoping"]
DoorsModule = Literal["none", "cockpit_door", "bulkhead_door", "both"]
PaletteTheme = Literal["desert", "navy", "green"]

AIRFRAME_MODULES: tuple[AirframeModule, ...] = ("box", "bellypod", "humpback", "stretch")
GEAR_MODULES: tuple[GearModule, ...] = ("fixed_strut", "oleo_telescoping")
DOORS_MODULES: tuple[DoorsModule, ...] = ("none", "cockpit_door", "bulkhead_door", "both")
PALETTE_THEMES: tuple[PaletteTheme, ...] = ("desert", "navy", "green")

BLADE_MIN, BLADE_MAX = 2, 8
SEAT_MIN, SEAT_MAX = 4, 18
NOZZLE_MIN, NOZZLE_MAX = 1, 4

# Fuselage envelope perturbation (spec: 长/宽/高微扰 ±10% around the baseline).
# Each axis is sampled/clamped independently. Length scales about x=0, width
# about the centerline y=0, and height about the belly bottom z0 = BELLY_Z +
# Z_LIFT so the wheels keep touching the ground (gear/ground-clearance
# guardrails hold for free under a belly-anchored height scale).
SCALE_MIN, SCALE_MAX = 0.90, 1.10

# ---------------------------------------------------------------------------
# Palettes
# ---------------------------------------------------------------------------
_COMMON = {
    "glass": (0.05, 0.05, 0.07, 1.0),
    "rotor_black": (0.07, 0.07, 0.08, 1.0),
    "tip": (0.72, 0.82, 0.28, 1.0),
    "metal": (0.22, 0.22, 0.24, 1.0),
    "rubber": (0.05, 0.05, 0.05, 1.0),
    "cabin_floor": (0.16, 0.16, 0.18, 1.0),
    "seat": (0.30, 0.34, 0.22, 1.0),
    "seat_frame": (0.18, 0.18, 0.19, 1.0),
    "exhaust": (0.17, 0.18, 0.19, 1.0),
    "belly": (0.66, 0.66, 0.68, 1.0),
}
PALETTES: dict[PaletteTheme, dict[str, tuple[float, float, float, float]]] = {
    "desert": {
        "body": (0.78, 0.66, 0.44, 1.0),
        "body_dark": (0.57, 0.46, 0.30, 1.0),
        "frame": (0.45, 0.36, 0.22, 1.0),
        **_COMMON,
    },
    "navy": {
        "body": (0.20, 0.26, 0.40, 1.0),
        "body_dark": (0.12, 0.16, 0.28, 1.0),
        "frame": (0.10, 0.13, 0.24, 1.0),
        **_COMMON,
    },
    "green": {
        "body": (0.28, 0.46, 0.22, 1.0),
        "body_dark": (0.18, 0.32, 0.14, 1.0),
        "frame": (0.14, 0.26, 0.11, 1.0),
        **_COMMON,
    },
}

# ---------------------------------------------------------------------------
# Fixed airframe constants (baseline CH-47; stretch shifts the aft block)
# ---------------------------------------------------------------------------
Z_LIFT = 0.30
NOSE_X = 8.15
CABIN_TOP = 3.05
BELLY_Z = 0.62
CABIN_W = 2.30
TAIL_BOTTOM_Z = 2.15
UPSWEEP_ANG = 0.47169

RAMP_LEN = 2.95
RAMP_W = 1.85
RAMP_T = 0.07
RAMP_CLEAR = 0.02
RAMP_OPEN = 0.74

ROTOR_R = 8.95
FRONT_TILT = math.radians(9.0)
REAR_TILT = math.radians(4.0)
HUB_STANDOFF = 0.30
HUB_R = 0.32
HUB_H = 0.36

WHEEL_R = 0.28
WHEEL_W = 0.18
GEAR_Y = 1.42
FRONT_GEAR_X = 3.45
SPONSON_R = 0.45
SPONSON_Y = 1.42
SPONSON_Z = 1.00 + Z_LIFT


def _clampi(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(v)))


def _clampf(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(v)))


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class TandemRotorHelicopterConfig:
    airframe_module: AirframeModule = "box"
    gear_module: GearModule = "fixed_strut"
    doors_module: DoorsModule = "none"
    palette_theme: PaletteTheme = "desert"
    pointed_nose: bool = False
    front_blade_count: int = 3
    rear_blade_count: int = 3
    seat_count: int = 9
    exhaust_nozzle_count: int = 2
    fuselage_length_scale: float = 1.0
    fuselage_width_scale: float = 1.0
    fuselage_height_scale: float = 1.0


@dataclass(frozen=True)
class ResolvedTandemRotorHelicopterConfig:
    airframe_module: AirframeModule
    gear_module: GearModule
    doors_module: DoorsModule
    palette_theme: PaletteTheme
    pointed_nose: bool
    front_blade_count: int
    rear_blade_count: int
    seat_count: int
    exhaust_nozzle_count: int
    len_scale: float
    width_scale: float
    height_scale: float
    # derived
    stretch: float
    tail_x: float
    upsweep_x0: float
    ramp_hinge: tuple[float, float, float]
    front_mount: tuple[float, float, float]
    rear_mount: tuple[float, float, float]
    rear_gear_x: float
    sponson_x0: float
    sponson_len: float
    exh_cx: float
    rear_pylon_cx: float
    front_hub: tuple[float, float, float]
    rear_hub: tuple[float, float, float]
    cockpit_flank_y: float
    palette: dict[str, tuple[float, float, float, float]]


def config_from_seed(seed: int) -> TandemRotorHelicopterConfig:
    rng = random.Random(seed)
    airframe = rng.choice(AIRFRAME_MODULES)
    gear = rng.choice(GEAR_MODULES)
    doors = rng.choice(DOORS_MODULES)
    palette = rng.choice(PALETTE_THEMES)
    return TandemRotorHelicopterConfig(
        airframe_module=airframe,
        gear_module=gear,
        doors_module=doors,
        palette_theme=palette,
        pointed_nose=(rng.random() < 0.5),
        front_blade_count=rng.randint(BLADE_MIN, BLADE_MAX),
        rear_blade_count=rng.randint(BLADE_MIN, BLADE_MAX),
        seat_count=rng.randint(SEAT_MIN, SEAT_MAX),
        exhaust_nozzle_count=rng.randint(NOZZLE_MIN, NOZZLE_MAX),
        fuselage_length_scale=rng.uniform(SCALE_MIN, SCALE_MAX),
        fuselage_width_scale=rng.uniform(SCALE_MIN, SCALE_MAX),
        fuselage_height_scale=rng.uniform(SCALE_MIN, SCALE_MAX),
    )


def _axis_dir(tilt: float) -> tuple[float, float, float]:
    return (math.sin(tilt), 0.0, math.cos(tilt))


def _hub_center(mount: tuple[float, float, float], tilt: float) -> tuple[float, float, float]:
    a = _axis_dir(tilt)
    return (
        mount[0] + HUB_STANDOFF * a[0],
        mount[1] + HUB_STANDOFF * a[1],
        mount[2] + HUB_STANDOFF * a[2],
    )


# ---------------------------------------------------------------------------
# Fuselage envelope transform (belly-anchored height, centerline width)
# ---------------------------------------------------------------------------
def _sx(r: ResolvedTandemRotorHelicopterConfig, x: float) -> float:
    return x * r.len_scale


def _sy(r: ResolvedTandemRotorHelicopterConfig, y: float) -> float:
    return y * r.width_scale


def _sz(r: ResolvedTandemRotorHelicopterConfig, z: float) -> float:
    z0 = BELLY_Z + Z_LIFT
    return z0 + (z - z0) * r.height_scale


def _sp(
    r: ResolvedTandemRotorHelicopterConfig, p: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Scale a world-space point: x about 0, y about the centerline, z about belly."""
    return (_sx(r, p[0]), _sy(r, p[1]), _sz(r, p[2]))


def _sl(
    r: ResolvedTandemRotorHelicopterConfig, p: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Scale a part-local offset (relative to a joint anchor) per axis.

    Because the world transform is affine with the same per-axis slope, a local
    offset relative to any anchor scales by (len, width, height) directly.
    """
    return (p[0] * r.len_scale, p[1] * r.width_scale, p[2] * r.height_scale)


def _ramp_geom(r: ResolvedTandemRotorHelicopterConfig):
    """Scaled upswept-tail ramp geometry.

    The upswept underside's slope is tan(UPSWEEP_ANG) * height/length under the
    envelope transform, so the rear cargo ramp (and the doorway that opens it)
    must adopt that scaled pitch and run length to stay flush — keeping the
    baseline angle would drive the plate straight into the hull. Returns the
    scaled pitch ``phi`` (about Y), ramp length ``L`` and width ``W``, and the
    run-length factor ``seg`` (scaled / baseline along the underside).
    """
    ct, st = math.cos(UPSWEEP_ANG), math.sin(UPSWEEP_ANG)
    dx, dz = ct * r.len_scale, st * r.height_scale
    phi = math.atan2(dz, dx)
    seg = math.hypot(dx, dz)
    return phi, RAMP_LEN * seg, RAMP_W * r.width_scale, seg


def resolve_config(
    config: TandemRotorHelicopterConfig,
) -> ResolvedTandemRotorHelicopterConfig:
    airframe = config.airframe_module
    if airframe not in AIRFRAME_MODULES:
        raise ValueError(f"Unsupported airframe_module: {airframe}")
    gear = config.gear_module if config.gear_module in GEAR_MODULES else "fixed_strut"
    doors = config.doors_module if config.doors_module in DOORS_MODULES else "none"
    palette_theme = config.palette_theme if config.palette_theme in PALETTES else "desert"

    stretch = 5.0 if airframe == "stretch" else 0.0
    tail_x = -8.0 - stretch
    upsweep_x0 = -5.0 - stretch
    ramp_hinge = (-4.95 - stretch, 0.0, 0.585 + Z_LIFT)
    # Rotor mounts sit ON the pylon top surfaces so the spin-joint origin is
    # within tol of real pylon hardware (origin-distance gate is point->surface,
    # so an origin deep inside a solid reads as far); the mast straddles the top.
    front_mount = (5.90, 0.0, 3.30 + Z_LIFT)
    rear_mount = (-6.45 - stretch, 0.0, 5.35 + Z_LIFT)
    rear_gear_x = -4.25 - stretch
    sponson_x0 = -4.6 - stretch
    sponson_len = 8.6 + stretch
    exh_cx = -7.95 - stretch
    rear_pylon_cx = -6.55 - stretch

    # bellypod hull is a touch wider over the cockpit flank
    cockpit_flank_y = 1.22 if airframe == "bellypod" else 1.15

    palette = dict(PALETTES[palette_theme])

    return ResolvedTandemRotorHelicopterConfig(
        airframe_module=airframe,
        gear_module=gear,
        doors_module=doors,
        palette_theme=palette_theme,
        pointed_nose=bool(config.pointed_nose),
        front_blade_count=_clampi(config.front_blade_count, BLADE_MIN, BLADE_MAX),
        rear_blade_count=_clampi(config.rear_blade_count, BLADE_MIN, BLADE_MAX),
        seat_count=_clampi(config.seat_count, SEAT_MIN, SEAT_MAX),
        exhaust_nozzle_count=_clampi(config.exhaust_nozzle_count, NOZZLE_MIN, NOZZLE_MAX),
        len_scale=_clampf(config.fuselage_length_scale, SCALE_MIN, SCALE_MAX),
        width_scale=_clampf(config.fuselage_width_scale, SCALE_MIN, SCALE_MAX),
        height_scale=_clampf(config.fuselage_height_scale, SCALE_MIN, SCALE_MAX),
        stretch=stretch,
        tail_x=tail_x,
        upsweep_x0=upsweep_x0,
        ramp_hinge=ramp_hinge,
        front_mount=front_mount,
        rear_mount=rear_mount,
        rear_gear_x=rear_gear_x,
        sponson_x0=sponson_x0,
        sponson_len=sponson_len,
        exh_cx=exh_cx,
        rear_pylon_cx=rear_pylon_cx,
        front_hub=_hub_center(front_mount, FRONT_TILT),
        rear_hub=_hub_center(rear_mount, REAR_TILT),
        cockpit_flank_y=cockpit_flank_y,
        palette=palette,
    )


# ---------------------------------------------------------------------------
# Geometry helpers (ported from the workbench sample sources)
# ---------------------------------------------------------------------------
def _rrect_yz(
    x: float, w: float, zb: float, zt: float,
    r_top: float, r_bot: float | None = None, n: int = 8,
    *, sc: ResolvedTandemRotorHelicopterConfig | None = None,
) -> LoftSection:
    """Rounded-rect loop in the YZ plane at station x (independent top/bot radii).

    The loop is built at the baseline envelope, then (if ``sc`` is given) the
    points are passed through the fuselage envelope transform so the hull scales
    anisotropically with the length/width/height knobs.
    """
    zb += Z_LIFT
    zt += Z_LIFT
    h = zt - zb
    r_top = min(r_top, 0.49 * w, 0.49 * h)
    if r_bot is None:
        r_bot = r_top
    r_bot = min(r_bot, 0.49 * w, 0.49 * h)
    cy_t = w / 2.0 - r_top
    cy_b = w / 2.0 - r_bot
    corners = (
        (cy_t, zt - r_top, 0.0, r_top),
        (-cy_t, zt - r_top, 90.0, r_top),
        (-cy_b, zb + r_bot, 180.0, r_bot),
        (cy_b, zb + r_bot, 270.0, r_bot),
    )
    pts: list[tuple[float, float, float]] = []
    for ccy, ccz, a0, cr in corners:
        for k in range(n):
            a = math.radians(a0 + 90.0 * k / n)
            pts.append((x, ccy + cr * math.cos(a), ccz + cr * math.sin(a)))
    if sc is not None:
        pts = [_sp(sc, p) for p in pts]
    return LoftSection(points=tuple(pts))


def _rrect_xy(
    z: float, cx: float, lx: float, wy: float, r: float, n: int = 7,
    *, sc: ResolvedTandemRotorHelicopterConfig | None = None,
) -> LoftSection:
    """Rounded-rect loop in the XY plane at height z (pylon cross-section)."""
    z += Z_LIFT
    r = min(r, 0.49 * lx, 0.49 * wy)
    hx = lx / 2.0 - r
    hy = wy / 2.0 - r
    corners = ((cx + hx, hy, 0.0), (cx - hx, hy, 90.0), (cx - hx, -hy, 180.0), (cx + hx, -hy, 270.0))
    pts = []
    for ccx, ccy, a0 in corners:
        for k in range(n):
            a = math.radians(a0 + 90.0 * k / n)
            pts.append((ccx + r * math.cos(a), ccy + r * math.sin(a), z))
    if sc is not None:
        pts = [_sp(sc, p) for p in pts]
    return LoftSection(points=tuple(pts))


def _hull_wire(sec: LoftSection):
    pts = [cq.Vector(float(px), float(py), float(pz)) for (px, py, pz) in sec.points]
    return cq.Wire.makePolygon(pts + [pts[0]])


def _capsule_x(radius: float, length: float, end_r: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).edges().fillet(end_r)


def _nose_sections(pointed: bool, t: float = 0.0):
    """Forward (nose) hull sections appended after the x=6.3 cabin station.

    Returns (outer, inner) lists. t is the wall inset for the inner shell.
    """
    if pointed:
        outer = [
            (6.9, 1.70, 0.85, 2.78, 0.28),
            (7.4, 0.90, 1.20, 2.25, 0.20),
            (7.9, 0.45, 1.45, 1.95, 0.12),
            (NOSE_X, 0.15, 1.60, 1.80, 0.05),
        ]
    else:
        outer = [
            (7.3, 2.05, 0.78, 2.85, 0.45),
            (7.9, 1.45, 1.02, 2.38, 0.50),
            (NOSE_X, 0.80, 1.30, 1.95, 0.32),
        ]
    inner = []
    for (x, w, zb, zt, r) in outer:
        iw = max(w - 2 * t, 0.06)
        if x >= NOSE_X - 1e-6 and pointed:
            inner.append((x, 0.06, 1.66, 1.74, 0.02))
        else:
            inner.append((x, iw, zb + t, zt - t, max(r - 0.05, 0.02)))
    return outer, inner


def _hull_sections(r: ResolvedTandemRotorHelicopterConfig):
    """Return (outer_sections, inner_sections) LoftSection lists for the hull."""
    T = 0.10
    af = r.airframe_module
    tail_x, up = r.tail_x, r.upsweep_x0

    # --- aft + mid cabin (airframe-specific), up to the x=6.3 station ---
    if af == "bellypod":
        # Aft sections (tail / -6.5 / upsweep belly) match the box family so the
        # upswept tail, cargo doorway and ramp stay aligned; only the mid cabin
        # bulges into the deep rounded pod.
        outer = [
            (tail_x, 1.75, TAIL_BOTTOM_Z, CABIN_TOP, 0.30),
            (-6.5, 2.15, 1.385, CABIN_TOP, 0.30),
            (up, CABIN_W, BELLY_Z, CABIN_TOP, 0.30),
            (-2.0, 2.55, 0.25, 3.30, 1.00),
            (2.0, 2.55, 0.25, 3.30, 1.00),
            (6.3, 2.45, 0.45, 3.15, 0.90),
        ]
        inner = [
            (tail_x, 1.75 - 2 * T, TAIL_BOTTOM_Z + T, CABIN_TOP - T, 0.20),
            (-6.5, 2.15 - 2 * T, 1.385 + T, CABIN_TOP - T, 0.20),
            (up, CABIN_W - 2 * T, BELLY_Z + T, CABIN_TOP - T, 0.20),
            (-2.0, 2.55 - 2 * T, 0.25 + T, 3.30 - T, 0.85),
            (2.0, 2.55 - 2 * T, 0.25 + T, 3.30 - T, 0.85),
            (6.3, 2.45 - 2 * T, 0.45 + T, 3.15 - T, 0.75),
        ]
        outer_r = [(x, w, zb, zt, rr, rr) for (x, w, zb, zt, rr) in outer]
        inner_r = [(x, w, zb, zt, rr, rr) for (x, w, zb, zt, rr) in inner]
    elif af == "humpback":
        outer_r = [
            (tail_x, 1.75, TAIL_BOTTOM_Z, CABIN_TOP, 0.55, 0.30),
            (-6.5, 2.15, 1.385, CABIN_TOP, 0.70, 0.35),
            (up, CABIN_W, BELLY_Z, CABIN_TOP, 0.80, 0.30),
            (-2.0, CABIN_W, BELLY_Z, CABIN_TOP, 0.80, 0.30),
            (2.0, CABIN_W, BELLY_Z, CABIN_TOP, 0.80, 0.30),
            (6.3, CABIN_W, BELLY_Z, CABIN_TOP, 0.80, 0.30),
        ]
        inner_r = [
            (tail_x, 1.75 - 2 * T, TAIL_BOTTOM_Z + T, CABIN_TOP - T, 0.45, 0.20),
            (-6.5, 2.15 - 2 * T, 1.385 + T, CABIN_TOP - T, 0.60, 0.25),
            (up, CABIN_W - 2 * T, BELLY_Z + T, CABIN_TOP - T, 0.70, 0.20),
            (-2.0, CABIN_W - 2 * T, BELLY_Z + T, CABIN_TOP - T, 0.70, 0.20),
            (2.0, CABIN_W - 2 * T, BELLY_Z + T, CABIN_TOP - T, 0.70, 0.20),
            (6.3, CABIN_W - 2 * T, BELLY_Z + T, CABIN_TOP - T, 0.70, 0.20),
        ]
    else:  # box or stretch
        cabin_xs = [-2.0, 2.0, 6.3]
        if r.stretch > 0.01:
            cabin_xs = [up + 4.0, -2.0, 2.0, 6.3]
        outer = [
            (tail_x, 1.75, TAIL_BOTTOM_Z, CABIN_TOP, 0.30),
            (tail_x + 1.5, 2.15, 1.385, CABIN_TOP, 0.30),
            (up, CABIN_W, BELLY_Z, CABIN_TOP, 0.30),
        ] + [(x, CABIN_W, BELLY_Z, CABIN_TOP, 0.30) for x in cabin_xs]
        inner = [
            (tail_x, 1.75 - 2 * T, TAIL_BOTTOM_Z + T, CABIN_TOP - T, 0.20),
            (tail_x + 1.5, 2.15 - 2 * T, 1.385 + T, CABIN_TOP - T, 0.20),
            (up, CABIN_W - 2 * T, BELLY_Z + T, CABIN_TOP - T, 0.20),
        ] + [(x, CABIN_W - 2 * T, BELLY_Z + T, CABIN_TOP - T, 0.20) for x in cabin_xs]
        outer_r = [(x, w, zb, zt, rr, rr) for (x, w, zb, zt, rr) in outer]
        inner_r = [(x, w, zb, zt, rr, rr) for (x, w, zb, zt, rr) in inner]

    # --- nose (shared, toggled) ---
    nose_out, nose_in = _nose_sections(r.pointed_nose, t=T)
    outer_r += [(x, w, zb, zt, rr, rr) for (x, w, zb, zt, rr) in nose_out]
    inner_r += [(x, w, zb, zt, rr, rr) for (x, w, zb, zt, rr) in nose_in]

    outer_sec = [_rrect_yz(x, w, zb, zt, rt, rbo, sc=r) for (x, w, zb, zt, rt, rbo) in outer_r]
    inner_sec = [_rrect_yz(x, w, zb, zt, rt, rbo, sc=r) for (x, w, zb, zt, rt, rbo) in inner_r]
    return outer_sec, inner_sec


# ---------------------------------------------------------------------------
# Fuselage
# ---------------------------------------------------------------------------
def _flank_y(x: float) -> float:
    """Left (+Y) hull half-width at a cockpit-region station x (box family)."""
    if x <= 6.3:
        return 1.15
    return 1.15 - 0.125 * (x - 6.3)


def _build_fuselage(model: ArticulatedObject, r: ResolvedTandemRotorHelicopterConfig):
    P = r.palette
    body = model.material("body", rgba=P["body"])
    body_dark = model.material("body_dark", rgba=P["body_dark"])
    glass = model.material("glass", rgba=P["glass"])
    metal = model.material("metal", rgba=P["metal"])
    exh_mat = model.material("exhaust", rgba=P["exhaust"])
    cabin_floor_mat = model.material("cabin_floor", rgba=P["cabin_floor"])
    seat_mat = model.material("seat", rgba=P["seat"])
    seat_frame_mat = model.material("seat_frame", rgba=P["seat_frame"])
    frame_mat = model.material("frame", rgba=P["frame"])

    fus = model.part("fuselage")

    outer_sec, inner_sec = _hull_sections(r)
    hull_solid = cq.Solid.makeLoft([_hull_wire(s) for s in outer_sec], ruled=True)
    inner_solid = cq.Solid.makeLoft([_hull_wire(s) for s in inner_sec], ruled=True)
    hull_solid = hull_solid.cut(inner_solid)

    # Rear cargo doorway cut (ramp opening in the upswept underside). The cut is
    # placed at the scaled hinge foot and its extents scale per-axis so it still
    # pierces the (scaled) tail underside.
    # The cargo ramp is a tilted subsystem: it adopts the scaled upsweep pitch
    # (_ramp_geom) so the opening and plate follow the scaled underside slope.
    # The doorway keeps its baseline clearances relative to the (scaled) ramp.
    phi, ramp_len_s, ramp_w_s, _seg = _ramp_geom(r)
    ct, st = math.cos(phi), math.sin(phi)
    du = (-ct, 0.0, st)
    mid = ramp_len_s / 2.0
    rh = r.ramp_hinge
    hinge_w = _sp(r, rh)
    foot_c = (hinge_w[0] + mid * du[0], 0.0, hinge_w[2] + mid * du[2])
    od_l, od_w, od_t = ramp_len_s - 0.30, ramp_w_s - 0.12, 0.35
    doorway = cq.Solid.makeBox(od_l, od_w, od_t).translate((-od_l / 2.0, -od_w / 2.0, -od_t / 2.0))
    doorway = doorway.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), phi * 180.0 / math.pi)
    doorway = doorway.translate(cq.Vector(*foot_c))
    hull_solid = hull_solid.cut(doorway)

    # Cockpit crew-door opening (left flank) when that door module is active.
    has_cockpit = r.doors_module in ("cockpit_door", "both")
    if has_cockpit:
        op_l, op_w, op_h = 0.96 * r.len_scale, 0.70 * r.width_scale, 1.21 * r.height_scale
        opening = cq.Solid.makeBox(op_l, op_w, op_h).translate(_sp(r, (5.92, 0.78, 1.27)))
        hull_solid = hull_solid.cut(opening)

    fus.visual(mesh_from_cadquery(cq.Workplane("XY").add(hull_solid), "hull"), material=body, name="hull")

    # Airframe-side ramp hinge tube at the tail belly (gives the ramp hinge
    # joint origin real parent hardware to seat against).
    fus.visual(
        Cylinder(0.07, RAMP_W * r.width_scale),
        origin=Origin(xyz=_sp(r, (rh[0], 0.0, rh[2])), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="ramp_hinge_mount",
    )

    # Dorsal hump for the humpback airframe.
    if r.airframe_module == "humpback":
        # Bottoms dropped well into the cabin roof so the hump embeds into the
        # hull (overlaps) rather than perching on the surface as a float island.
        hump_sections = (
            _rrect_yz(-3.6, 1.25, 2.40, 3.30, 0.28, 0.18, sc=r),
            _rrect_yz(-4.2, 1.55, 2.40, 4.15, 0.55, 0.25, sc=r),
            _rrect_yz(-4.9, 1.65, 2.40, 4.60, 0.65, 0.25, sc=r),
            _rrect_yz(-5.6, 1.55, 2.40, 4.30, 0.55, 0.25, sc=r),
            _rrect_yz(-6.4, 1.35, 2.40, 3.70, 0.40, 0.20, sc=r),
        )
        hump_solid = cq.Solid.makeLoft([_hull_wire(s) for s in hump_sections], ruled=True)
        fus.visual(
            mesh_from_cadquery(cq.Workplane("XY").add(hump_solid), "dorsal_hump"),
            material=body,
            name="dorsal_hump",
        )

    # Glazed cockpit canopy (follows the nose taper if pointed).
    # Canopy is set slightly INSIDE the hull width at each station so the whole
    # glazing band embeds into (overlaps) the hull -> no floating components.
    if r.pointed_nose:
        # The hull is a thin (~0.10) lofted shell. Center the glazing band in
        # the shell-wall band at each station (half-width midway between the
        # inner and outer surfaces) so the canopy's side faces always overlap
        # the wall — not the hollow cavity (floats) nor poking out (rim island).
        canopy_sections = (
            _rrect_yz(6.90, 1.60, 1.45, 2.72, 0.28, sc=r),
            _rrect_yz(7.40, 0.80, 1.30, 2.19, 0.18, sc=r),
            _rrect_yz(7.80, 0.44, 1.45, 1.95, 0.10, sc=r),
        )
    else:
        canopy_sections = (
            _rrect_yz(6.90, 2.06, 1.30, 2.96, 0.38, sc=r),
            _rrect_yz(7.30, 1.92, 1.20, 2.86, 0.42, sc=r),
            _rrect_yz(7.90, 1.34, 1.05, 2.38, 0.40, sc=r),
        )
    # Build the canopy with the same robust OpenCASCADE loft as the hull/hump:
    # the custom section_loft tessellator fragments into disconnected mesh
    # islands when the rounded-rect points are scaled anisotropically.
    canopy_solid = cq.Solid.makeLoft([_hull_wire(s) for s in canopy_sections], ruled=True)
    fus.visual(
        mesh_from_cadquery(cq.Workplane("XY").add(canopy_solid), "cockpit_canopy"),
        material=glass,
        name="cockpit_canopy",
    )

    # Drive-shaft spine linking the two pylons.
    spine_cx = _sx(r, (5.20 + r.rear_mount[0]) / 2.0)
    spine_len = (5.20 - r.rear_mount[0]) * r.len_scale
    fus.visual(
        Cylinder(0.30, spine_len),
        origin=Origin(xyz=(spine_cx, 0.0, _sz(r, 3.00 + Z_LIFT)), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body,
        name="spine_fairing",
    )

    # Front pylon (low fairing above the cockpit).
    front_pylon_sections = (
        _rrect_xy(2.95, 5.90, 1.45, 1.50, 0.20, sc=r),
        _rrect_xy(3.30, 5.90, 1.05, 1.00, 0.18, sc=r),
    )
    fus.visual(
        mesh_from_geometry(section_loft(SectionLoftSpec(sections=front_pylon_sections, ruled=True)), "front_pylon"),
        material=body,
        name="front_pylon",
    )

    # Rear pylon (tall tower at the tail, shifted aft by stretch).
    rpx = r.rear_pylon_cx
    rear_pylon_sections = (
        _rrect_xy(2.90, rpx, 2.90, 1.70, 0.40, sc=r),
        _rrect_xy(4.20, rpx - 0.20, 2.30, 1.35, 0.40, sc=r),
        _rrect_xy(5.35, rpx + 0.10, 1.60, 1.05, 0.35, sc=r),
    )
    fus.visual(
        mesh_from_geometry(section_loft(SectionLoftSpec(sections=rear_pylon_sections, ruled=True)), "rear_pylon"),
        material=body,
        name="rear_pylon",
    )

    # Twin engine nacelles flanking the rear pylon base. Seat them inboard
    # enough to keep a solid overlap with the (scaled) pylon flank across the
    # full width range — at the old 1.22 spacing the overlap was ~0 at baseline
    # and opened into a gap once the width knob widened the airframe.
    engine = _capsule_x(0.42, 2.70 * r.len_scale, 0.25)
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        fus.visual(
            mesh_from_cadquery(engine.translate(_sp(r, (-7.65 - r.stretch, sign * 1.05, 3.25 + Z_LIFT))), f"{side}_engine_nacelle"),
            material=body_dark,
            name=f"{side}_engine_nacelle",
        )

    # Rear exhaust: cover plate with N bored nozzles. Plate width/height follow
    # the width/height knobs; lengthwise extrudes follow the length knob.
    EXH_CX, EXH_CZ, EXH_R = r.exh_cx, 2.85, 0.20
    nz = r.exhaust_nozzle_count
    span = 0.40 * r.width_scale
    if nz == 1:
        ys = [0.0]
    else:
        ys = [(-span + 2 * span * i / (nz - 1)) for i in range(nz)]
    plate_w = max(1.20, 0.5 + 0.5 * nz) * r.width_scale
    excx, excz = _sx(r, EXH_CX), _sz(r, EXH_CZ)
    cover = (
        cq.Workplane("YZ").rect(plate_w, 0.70 * r.height_scale).extrude(0.10 * r.len_scale).edges("|X").fillet(0.12)
        .translate((excx - 0.05 * r.len_scale, 0.0, excz))
    )
    for hy in ys:
        cover = cover.cut(cq.Workplane("YZ").circle(EXH_R).extrude(0.60 * r.len_scale).translate((excx - 0.30 * r.len_scale, hy, excz)))
    for hy in ys:
        cover = cover.union(
            cq.Workplane("YZ").circle(EXH_R).circle(EXH_R - 0.07).extrude(-0.45 * r.len_scale)
            .translate((excx + 0.05 * r.len_scale, hy, excz))
        )
    fus.visual(mesh_from_cadquery(cover, "rear_exhaust"), material=exh_mat, name="rear_exhaust")

    # Side fuel sponsons.
    sponson = _capsule_x(SPONSON_R, r.sponson_len * r.len_scale, 0.30)
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        fus.visual(
            mesh_from_cadquery(sponson.translate(_sp(r, (r.sponson_x0, sign * SPONSON_Y, SPONSON_Z))), f"{side}_sponson"),
            material=body,
            name=f"{side}_sponson",
        )

    # Cabin windows on each flank. Placed at the airframe's actual flank
    # half-width and at a mid-cabin height (below any dome rounding), and made
    # thick enough to pierce the skin so they always rest on the hull surface.
    cabin_flank_y = 1.275 if r.airframe_module == "bellypod" else 1.15
    win_xs = [-3.4 + 1.2 * i for i in range(6)]
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        for i, wx in enumerate(win_xs):
            fus.visual(
                Cylinder(0.16, 0.12),
                origin=Origin(xyz=_sp(r, (wx, sign * (cabin_flank_y - 0.03), 1.80 + Z_LIFT)), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=glass,
                name=f"{side}_window_{i}",
            )
        # Cockpit side window (full-flank cabin station so it works for any nose;
        # skip left flank when the crew door replaces it).
        if not (has_cockpit and side == "left"):
            fus.visual(
                Box((0.55, 0.10, 0.55)),
                origin=Origin(xyz=_sp(r, (6.2, sign * (cabin_flank_y - 0.04), 2.10 + Z_LIFT))),
                material=glass,
                name=f"{side}_cockpit_window",
            )

    # Cargo cabin floor + troop seats (N per side).
    cab_x0 = r.upsweep_x0 + 0.7
    cab_x1 = 3.3
    floor_z = BELLY_Z + 0.05 + Z_LIFT
    fus.visual(
        Box(((cab_x1 - cab_x0) * r.len_scale, CABIN_W * r.width_scale, 0.10)),
        origin=Origin(xyz=_sp(r, ((cab_x0 + cab_x1) / 2.0, 0.0, floor_z))),
        material=cabin_floor_mat,
        name="cabin_floor",
    )
    pan_z = floor_z + 0.43
    # Seat the backs straddling the inner cabin wall (hull inner half-width =
    # CABIN_W/2 - T) so they stay embedded in the shell at every width scale.
    # The old 1.0 anchor left a ~0.02 gap that the wide+tall corner of the
    # envelope opened into a full seat-bank island.
    wall_y = CABIN_W / 2.0 - 0.10
    n_seat = r.seat_count
    seat_span = cab_x1 - cab_x0 - 0.6
    # Seats keep their baseline box sizes; only the wall anchor (sx along the
    # scaled cargo bay, wall_y at the scaled flank) moves. The pan's inboard
    # (0.24) and vertical (0.28) offsets stay baseline so the pan and back keep
    # touching regardless of the width/height knobs (avoids island splits).
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        for i in range(n_seat):
            sx = cab_x0 + 0.3 + (seat_span * i / max(1, n_seat - 1) if n_seat > 1 else seat_span / 2.0)
            seat_x = _sx(r, sx)
            wall_y_s = _sy(r, wall_y)
            seat_z = _sz(r, pan_z)
            fus.visual(
                Box((0.40, 0.06, 0.52)),
                origin=Origin(xyz=(seat_x, sign * wall_y_s, seat_z + 0.28)),
                material=seat_mat,
                name=f"{side}_seat_back_{i}",
            )
            fus.visual(
                Box((0.40, 0.46, 0.06)),
                origin=Origin(xyz=(seat_x, sign * (wall_y_s - 0.24), seat_z)),
                material=seat_mat,
                name=f"{side}_seat_pan_{i}",
            )

    # Internal bulkhead partition (with door cutout) for the bulkhead-door module.
    if r.doors_module in ("bulkhead_door", "both"):
        bhd_x, bhd_t, bhd_y = 3.0, 0.06, 1.00
        bhd_z1, door_hw, door_top = 3.05, 0.42, 2.35
        wall_h = _sz(r, bhd_z1) - _sz(r, floor_z)
        door_h = _sz(r, door_top) - _sz(r, floor_z)
        wall = cq.Solid.makeBox(bhd_t, 2 * bhd_y * r.width_scale, wall_h).translate(
            (_sx(r, bhd_x) - bhd_t / 2.0, -bhd_y * r.width_scale, _sz(r, floor_z))
        )
        wall = wall.cut(
            cq.Solid.makeBox(bhd_t + 0.04, 2 * door_hw * r.width_scale, door_h).translate(
                (_sx(r, bhd_x) - bhd_t / 2.0 - 0.02, -door_hw * r.width_scale, _sz(r, floor_z))
            )
        )
        fus.visual(mesh_from_cadquery(cq.Workplane("XY").add(wall), "cabin_bulkhead"), material=frame_mat, name="cabin_bulkhead")

    # Door frame trim for the crew door (conforming surround with the aperture cut out).
    if has_cockpit:
        def _yz_wire(x, ya, yb, z0, z1):
            pts = [(x, ya, z0), (x, yb, z0), (x, yb, z1), (x, ya, z1)]
            return cq.Wire.makePolygon([cq.Vector(*p) for p in pts] + [cq.Vector(*pts[0])])

        fxs = (5.95 - 0.06, 0.5 * (5.95 + 6.85), 6.85 + 0.06)
        frame = cq.Solid.makeLoft(
            [
                _yz_wire(
                    _sx(r, x), _sy(r, _flank_y(x) - 0.01), _sy(r, _flank_y(x) + 0.035),
                    _sz(r, 1.30 - 0.06), _sz(r, 2.45 + 0.06),
                )
                for x in fxs
            ],
            ruled=True,
        )
        cf_l, cf_w, cf_h = 0.91 * r.len_scale, 0.40 * r.width_scale, 1.19 * r.height_scale
        frame = frame.cut(cq.Solid.makeBox(cf_l, cf_w, cf_h).translate(_sp(r, (5.96, 0.85, 1.28))))
        fus.visual(mesh_from_cadquery(cq.Workplane("XY").add(frame), "door_frame"), material=body_dark, name="door_frame")

    return fus


# ---------------------------------------------------------------------------
# Rotors
# ---------------------------------------------------------------------------
def _build_rotor(model, fus, part_name, mount_world, hub_world, tilt, axis, base_angle, blade_count, P):
    # The spin joint origin sits at `mount_world` (on the scaled pylon top, real
    # hardware), so the child frame's z=0 is at the mount. The mast rises from
    # z=0 and the hub/blades sit at z=HUB_STANDOFF. The rotor itself is not part
    # of the fuselage envelope, so its blades/mast keep their baseline size.
    metal = "metal"
    rotor = model.part(part_name)
    rotor.visual(Cylinder(0.085, 0.50), origin=Origin(xyz=(0.0, 0.0, 0.20)), material=metal, name="mast")
    rotor.visual(Cylinder(HUB_R, HUB_H), origin=Origin(xyz=(0.0, 0.0, HUB_STANDOFF)), material=metal, name="hub")
    rotor.visual(Cylinder(0.16, 0.14), origin=Origin(xyz=(0.0, 0.0, HUB_STANDOFF + 0.24)), material=metal, name="hub_cap")
    blade_geom = section_loft(
        [
            [(0.20, -0.30, -0.050), (0.20, 0.30, -0.050), (0.20, 0.30, 0.050), (0.20, -0.30, 0.050)],
            [(ROTOR_R, -0.16, -0.026), (ROTOR_R, 0.16, -0.026), (ROTOR_R, 0.16, 0.026), (ROTOR_R, -0.16, 0.026)],
        ]
    )
    blade_mesh = mesh_from_geometry(blade_geom, f"{part_name}_blade")
    tip_r = 8.575
    for i in range(blade_count):
        ang = base_angle + i * 2.0 * math.pi / blade_count
        rotor.visual(blade_mesh, origin=Origin(xyz=(0.0, 0.0, HUB_STANDOFF), rpy=(0.0, 0.0, ang)), material="rotor_black", name=f"blade_{i}")
        rotor.visual(
            Box((0.75, 0.34, 0.058)),
            origin=Origin(xyz=(tip_r * math.cos(ang), tip_r * math.sin(ang), HUB_STANDOFF), rpy=(0.0, 0.0, ang)),
            material="tip",
            name=f"blade_tip_{i}",
        )
    model.articulation(
        f"{part_name}_spin",
        ArticulationType.CONTINUOUS,
        parent=fus,
        child=rotor,
        origin=Origin(xyz=mount_world, rpy=(0.0, tilt, 0.0)),
        axis=axis,
        motion_limits=MotionLimits(effort=9000.0, velocity=30.0),
    )
    return rotor


def _build_rotors(model, r, fus):
    model.material("rotor_black", rgba=r.palette["rotor_black"])
    model.material("tip", rgba=r.palette["tip"])
    _build_rotor(model, fus, "front_rotor", _sp(r, r.front_mount), r.front_hub, FRONT_TILT, (0.0, 0.0, 1.0), 0.0, r.front_blade_count, r.palette)
    _build_rotor(model, fus, "rear_rotor", _sp(r, r.rear_mount), r.rear_hub, REAR_TILT, (0.0, 0.0, -1.0), math.pi / 3.0, r.rear_blade_count, r.palette)


# ---------------------------------------------------------------------------
# Landing gear
# ---------------------------------------------------------------------------
def _gear_positions(r):
    # Length/width knobs move the gear in x/y so the wheels stay under the
    # (scaled) sponsons; height is belly-anchored so wheel z / ground contact
    # are unchanged and handled with the baseline z literals in _build_gear.
    return (
        (_sx(r, FRONT_GEAR_X), _sy(r, GEAR_Y), "front_left"),
        (_sx(r, FRONT_GEAR_X), _sy(r, -GEAR_Y), "front_right"),
        (_sx(r, r.rear_gear_x), _sy(r, GEAR_Y), "rear_left"),
        (_sx(r, r.rear_gear_x), _sy(r, -GEAR_Y), "rear_right"),
    )


def _build_gear(model, r, fus):
    metal, rubber = "metal", "rubber"
    model.material("rubber", rgba=r.palette["rubber"])
    oleo = r.gear_module == "oleo_telescoping"
    for gx, gy, gname in _gear_positions(r):
        sgn = 1.0 if gy > 0 else -1.0
        leg_y = gy - sgn * 0.22
        if oleo:
            # Prismatic origin at the sleeve entry (z=0.62, real hardware); the
            # piston child frame's z=0 is there, so rod/axle are shifted down.
            oz = 0.62
            fus.visual(Cylinder(0.075, 0.40), origin=Origin(xyz=(gx, leg_y, 0.80)), material=metal, name=f"{gname}_gear_strut")
            piston = model.part(f"{gname}_oleo")
            piston.visual(Cylinder(0.05, 0.60), origin=Origin(xyz=(0.0, 0.0, 0.52 - oz)), material=metal, name="rod")
            piston.visual(
                Cylinder(0.035, 0.30),
                origin=Origin(xyz=(0.0, sgn * 0.09, WHEEL_R - oz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=metal,
                name=f"{gname}_axle",
            )
            model.articulation(
                f"{gname}_oleo_slide",
                ArticulationType.PRISMATIC,
                parent=fus,
                child=piston,
                origin=Origin(xyz=(gx, leg_y, oz)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(effort=80.0, velocity=0.3, lower=0.0, upper=0.18),
            )
            wheel_parent = piston
            spin_origin = Origin(xyz=(0.0, sgn * 0.22, WHEEL_R - oz))
        else:
            fus.visual(Cylinder(0.06, 0.62), origin=Origin(xyz=(gx, leg_y, 0.62)), material=metal, name=f"{gname}_gear_strut")
            fus.visual(
                Cylinder(0.035, 0.30),
                origin=Origin(xyz=(gx, gy - sgn * 0.13, WHEEL_R), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=metal,
                name=f"{gname}_axle",
            )
            wheel_parent = fus
            spin_origin = Origin(xyz=(gx, gy, WHEEL_R))
        wheel = model.part(f"{gname}_wheel")
        wheel.visual(Cylinder(WHEEL_R, WHEEL_W), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=rubber, name="tire")
        wheel.visual(Cylinder(0.10, WHEEL_W + 0.02), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=metal, name="hub")
        model.articulation(
            f"{gname}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=wheel_parent,
            child=wheel,
            origin=spin_origin,
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=200.0, velocity=40.0),
        )


# ---------------------------------------------------------------------------
# Cargo ramp
# ---------------------------------------------------------------------------
def _build_ramp(model, r, fus):
    metal = "metal"
    ramp = model.part("cargo_ramp")
    # Adopt the scaled upsweep pitch / run length so the closed ramp lies flush
    # with the (anisotropically) scaled tail underside.
    phi, ramp_len_s, ramp_w_s, seg = _ramp_geom(r)
    ramp.visual(Cylinder(0.06, 1.60 * r.width_scale), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=metal, name="hinge_barrel")
    ct, st = math.cos(phi), math.sin(phi)
    d = (-ct, 0.0, st)
    nrm = (-st, 0.0, -ct)
    plate_off = RAMP_T / 2.0 + RAMP_CLEAR
    plate_c = (ramp_len_s / 2.0 * d[0] + plate_off * nrm[0], 0.0, ramp_len_s / 2.0 * d[2] + plate_off * nrm[2])
    ramp.visual(Box((ramp_len_s, ramp_w_s, RAMP_T)), origin=Origin(xyz=plate_c, rpy=(0.0, phi, 0.0)), material="body", name="ramp_plate")
    lip_run = 2.90 * seg
    lip_c = (lip_run * d[0] + plate_off * nrm[0], 0.0, lip_run * d[2] + plate_off * nrm[2])
    ramp.visual(Box((0.12, ramp_w_s, 0.08)), origin=Origin(xyz=lip_c, rpy=(0.0, phi, 0.0)), material="body_dark", name="ramp_lip")
    rib_off = plate_off + RAMP_T / 2.0 + 0.020
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        rib_c = (ramp_len_s / 2.0 * d[0] + rib_off * nrm[0], sign * 0.55 * r.width_scale, ramp_len_s / 2.0 * d[2] + rib_off * nrm[2])
        ramp.visual(Box((2.60 * seg, 0.08, 0.05)), origin=Origin(xyz=rib_c, rpy=(0.0, phi, 0.0)), material="body_dark", name=f"{side}_ramp_rib")
    model.articulation(
        "ramp_hinge",
        ArticulationType.REVOLUTE,
        parent=fus,
        child=ramp,
        origin=Origin(xyz=_sp(r, r.ramp_hinge)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=0.8, lower=0.0, upper=RAMP_OPEN),
    )
    return ramp


# ---------------------------------------------------------------------------
# Doors
# ---------------------------------------------------------------------------
def _build_cockpit_door(model, r, fus):
    body, glass, metal, body_dark = "body", "glass", "metal", "body_dark"
    DX0, DX1, DZ0, DZ1, DT = 5.95, 6.85, 1.30, 2.45, 0.05
    door = model.part("cockpit_door")
    HX, HY, HZ = DX0, _flank_y(DX0), DZ0

    def _yz_wire(x, ya, yb, z0, z1):
        pts = [(x, ya, z0), (x, yb, z0), (x, yb, z1), (x, ya, z1)]
        return cq.Wire.makePolygon([cq.Vector(*p) for p in pts] + [cq.Vector(*pts[0])])

    anchor = _sp(r, (HX, HY, HZ))
    dxs = (DX0, 0.5 * (DX0 + DX1), DX1)
    skin = cq.Solid.makeLoft(
        [
            _yz_wire(_sx(r, x), _sy(r, _flank_y(x) - DT), _sy(r, _flank_y(x)), _sz(r, DZ0), _sz(r, DZ1))
            for x in dxs
        ],
        ruled=True,
    )
    skin = skin.cut(
        cq.Solid.makeBox(0.50 * r.len_scale, 0.60 * r.width_scale, 0.45 * r.height_scale)
        .translate(_sp(r, (DX0 + 0.20, 0.80, DZ0 + 0.55)))
    )
    skin = skin.translate((-anchor[0], -anchor[1], -anchor[2]))
    door.visual(mesh_from_cadquery(cq.Workplane("XY").add(skin), "door_skin"), material=body, name="door_skin")
    gx = DX0 + 0.45
    door.visual(
        Box((0.50 * r.len_scale, 0.03 * r.width_scale, 0.45 * r.height_scale)),
        origin=Origin(xyz=_sl(r, (gx - HX, _flank_y(gx) - DT / 2.0 - HY, DZ0 + 0.775 - HZ))),
        material=glass,
        name="door_window",
    )
    # refined handle: escutcheon + grab bar on 2 posts + thumb latch
    hx = DX1 - 0.18
    face = _flank_y(hx)
    hz = DZ0 + 0.55

    def _hl(x, y, z):
        return _sl(r, (x - HX, y - HY, z - HZ))

    door.visual(Box((0.11 * r.len_scale, 0.025 * r.width_scale, 0.30 * r.height_scale)), origin=Origin(xyz=_hl(hx, face + 0.012, hz)), material=body_dark, name="door_handle_plate")
    for dz, post in ((0.105, "top"), (-0.105, "bot")):
        door.visual(
            Cylinder(0.013, 0.065 * r.width_scale),
            origin=Origin(xyz=_hl(hx, face + 0.04, hz + dz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"door_handle_post_{post}",
        )
    door.visual(Cylinder(0.018, 0.25 * r.width_scale), origin=Origin(xyz=_hl(hx, face + 0.066, hz)), material=metal, name="door_handle")
    door.visual(Cylinder(0.035, (DZ1 - DZ0 - 0.10) * r.height_scale), origin=Origin(xyz=_sl(r, (0.0, 0.0, 0.5 * (DZ1 - DZ0)))), material=metal, name="hinge_knuckle")
    model.articulation(
        "cockpit_door_hinge",
        ArticulationType.REVOLUTE,
        parent=fus,
        child=door,
        origin=Origin(xyz=anchor),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=1.30),
    )
    return door


def _build_bulkhead_door(model, r, fus):
    body, glass, metal = "body", "glass", "metal"
    floor_z = BELLY_Z + 0.05 + Z_LIFT
    BHD_X, DOOR_HW, DOOR_TOP = 3.0, 0.42, 2.35
    bdoor = model.part("bulkhead_door")
    BX, BY, BZ = BHD_X, DOOR_HW, floor_z
    anchor = _sp(r, (BX, BY, BZ))
    DBOT, DT = floor_z + 0.05, DOOR_TOP - 0.03
    # Panel thickness (X) scales with the length knob to match the handle's
    # X-offsets (also length-scaled), so the handle stays welded to the panel.
    panel = cq.Solid.makeBox(0.05 * r.len_scale, (2 * DOOR_HW - 0.05) * r.width_scale, (DT - DBOT) * r.height_scale).translate(
        _sp(r, (BHD_X - 0.025, -(DOOR_HW - 0.025), DBOT))
    )
    panel = panel.cut(
        cq.Solid.makeBox(0.12 * r.len_scale, 0.34 * r.width_scale, 0.34 * r.height_scale)
        .translate(_sp(r, (BHD_X - 0.06, -0.27, DOOR_TOP - 0.62)))
    )
    panel = panel.translate((-anchor[0], -anchor[1], -anchor[2]))
    bdoor.visual(mesh_from_cadquery(cq.Workplane("XY").add(panel), "door_panel"), material=body, name="door_panel")
    bdoor.visual(
        Box((0.03 * r.len_scale, 0.34 * r.width_scale, 0.34 * r.height_scale)),
        origin=Origin(xyz=_sl(r, (BHD_X - BX, -0.10 - BY, DOOR_TOP - 0.45 - BZ))),
        material=glass,
        name="door_vision_window",
    )
    bdoor.visual(
        Cylinder(0.045, 0.02 * r.len_scale),
        origin=Origin(xyz=_sl(r, (BHD_X + 0.035 - BX, -0.30 - BY, DBOT + 0.85 - BZ)), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="door_handle_rose",
    )
    bdoor.visual(Box((0.03 * r.len_scale, 0.17 * r.width_scale, 0.025 * r.height_scale)), origin=Origin(xyz=_sl(r, (BHD_X + 0.06 - BX, -0.355 - BY, DBOT + 0.85 - BZ))), material=metal, name="door_handle")
    bdoor.visual(Cylinder(0.03, (DT - DBOT) * r.height_scale), origin=Origin(xyz=_sl(r, (0.0, 0.0, 0.5 * (DT - DBOT))), rpy=(0.0, 0.0, 0.0)), material=metal, name="hinge_knuckle")
    model.articulation(
        "bulkhead_door_hinge",
        ArticulationType.REVOLUTE,
        parent=fus,
        child=bdoor,
        origin=Origin(xyz=anchor),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=1.40),
    )
    return bdoor


# ---------------------------------------------------------------------------
# Top-level build
# ---------------------------------------------------------------------------
def build_tandem_rotor_helicopter(
    config: TandemRotorHelicopterConfig,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name="tandem_rotor_helicopter", assets=assets)
    fus = _build_fuselage(model, r)
    _build_rotors(model, r, fus)
    _build_gear(model, r, fus)
    _build_ramp(model, r, fus)
    if r.doors_module in ("cockpit_door", "both"):
        _build_cockpit_door(model, r, fus)
    if r.doors_module in ("bulkhead_door", "both"):
        _build_bulkhead_door(model, r, fus)
    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_tandem_rotor_helicopter(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_tandem_rotor_helicopter(config_from_seed(seed), assets=assets)


def slot_choices_for_config(r: ResolvedTandemRotorHelicopterConfig) -> list[tuple[str, str]]:
    return [
        ("airframe", r.airframe_module),
        ("landing_gear", r.gear_module),
        ("doors", r.doors_module),
        ("front_rotor", f"n{r.front_blade_count}"),
        ("rear_rotor", f"n{r.rear_blade_count}"),
        ("nose", "pointed" if r.pointed_nose else "blunt"),
    ]


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tandem_rotor_helicopter_tests(
    object_model: ArticulatedObject,
    config: TandemRotorHelicopterConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    fus = object_model.get_part("fuselage")
    front_rotor = object_model.get_part("front_rotor")
    rear_rotor = object_model.get_part("rear_rotor")

    part_names = {p.name for p in object_model.parts}
    ctx.check("fuselage_present", "fuselage" in part_names, "missing fuselage")
    ctx.check("front_rotor_present", "front_rotor" in part_names, "missing front_rotor")
    ctx.check("rear_rotor_present", "rear_rotor" in part_names, "missing rear_rotor")
    ctx.check("cargo_ramp_present", "cargo_ramp" in part_names, "missing cargo_ramp")
    ctx.check("no_tail_rotor", "tail_rotor" not in part_names, "tandem heli must not have a tail rotor")

    front_spin = object_model.get_articulation("front_rotor_spin")
    rear_spin = object_model.get_articulation("rear_rotor_spin")
    ctx.check(
        "rotors_continuous",
        front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS,
        "both tandem rotors must spin on continuous joints",
    )
    ctx.check(
        "rear_hub_above_front",
        1.5 <= (rear_spin.origin.xyz[2] - front_spin.origin.xyz[2]) <= 2.6,
        "rear rotor hub must sit clearly above the front (tandem signature)",
    )
    ctx.check(
        "tandem_hub_separation",
        (front_spin.origin.xyz[0] - rear_spin.origin.xyz[0]) >= 10.0,
        "rotor hubs must sit at opposite ends of the fuselage",
    )

    # blade-count multiplicity
    for rotor, part, count in (("front", front_rotor, r.front_blade_count), ("rear", rear_rotor, r.rear_blade_count)):
        n = len([v for v in part.visuals if v.name.startswith("blade_") and not v.name.startswith("blade_tip_")])
        ctx.check(f"{rotor}_blade_count", n == count, f"{rotor} rotor must have {count} blades, got {n}")

    # exhaust + seats multiplicity
    fus_vis = {v.name for v in fus.visuals}
    seats = len([v for v in fus.visuals if v.name.startswith("left_seat_pan_")])
    ctx.check("seat_count", seats == r.seat_count, f"expected {r.seat_count} seat rows/side, got {seats}")

    # intentional coaxial overlaps: rotor masts seat through pylon/hull tops
    ctx.allow_overlap(front_rotor, fus, elem_a="mast", elem_b="front_pylon", reason="Front mast seats through the front pylon bearing; coaxial with the spin axis.")
    ctx.allow_overlap(front_rotor, fus, elem_a="mast", elem_b="hull", reason="Front mast root passes the cabin-top skin; coaxial with the spin axis.")
    ctx.allow_overlap(front_rotor, fus, elem_a="hub", elem_b="front_pylon", reason="Front rotor head seats on the pylon/transmission fairing top; coaxial.")
    ctx.allow_overlap(rear_rotor, fus, elem_a="mast", elem_b="rear_pylon", reason="Rear mast seats through the rear pylon bearing; coaxial with the spin axis.")
    ctx.allow_overlap(rear_rotor, fus, elem_a="mast", elem_b="hull", reason="Rear mast root passes the spine skin; coaxial with the spin axis.")
    ctx.allow_overlap(rear_rotor, fus, elem_a="hub", elem_b="rear_pylon", reason="Rear rotor head seats on the pylon/transmission fairing top; coaxial.")

    # gear: wheels + (oleo) prismatic
    oleo = r.gear_module == "oleo_telescoping"
    for g in ("front_left", "front_right", "rear_left", "rear_right"):
        wp = object_model.get_part(f"{g}_wheel")
        axle_owner = object_model.get_part(f"{g}_oleo") if oleo else fus
        ctx.allow_overlap(wp, axle_owner, elem_a="hub", elem_b=f"{g}_axle", reason=f"Axle through the {g} hub bore; coaxial with the spin axis.")
        ctx.allow_overlap(wp, axle_owner, elem_a="tire", elem_b=f"{g}_axle", reason=f"Axle through the {g} tire bore; coaxial with the spin axis.")
        spin = object_model.get_articulation(f"{g}_wheel_spin")
        ctx.check(f"{g}_wheel_spins", spin.articulation_type == ArticulationType.CONTINUOUS, f"{g} wheel needs a continuous spin")
        if oleo:
            ctx.allow_overlap(object_model.get_part(f"{g}_oleo"), fus, elem_a="rod", elem_b=f"{g}_gear_strut", reason=f"{g} oleo rod slides inside the fixed sleeve; coaxial with the prismatic axis.")
            slide = object_model.get_articulation(f"{g}_oleo_slide")
            ctx.check(f"{g}_oleo_prismatic", slide.articulation_type == ArticulationType.PRISMATIC and (slide.motion_limits.upper or 0.0) >= 0.10, f"{g} leg must be a prismatic oleo strut")

    # ramp hinge embed
    ramp = object_model.get_part("cargo_ramp")
    ctx.allow_overlap(ramp, fus, elem_a="hinge_barrel", elem_b="hull", reason="Ramp hinge barrel embeds into the hull tail edge; on-axis, pose-invariant.")
    ctx.allow_overlap(ramp, fus, elem_a="hinge_barrel", elem_b="ramp_hinge_mount", reason="Ramp hinge barrel rides on the airframe-side hinge tube; coaxial with the hinge axis.")
    ctx.allow_overlap(ramp, fus, elem_a="ramp_plate", elem_b="ramp_hinge_mount", reason="Ramp plate root wraps the hinge tube at the pivot; coaxial with the hinge axis.")

    # doors
    if r.doors_module in ("cockpit_door", "both"):
        door = object_model.get_part("cockpit_door")
        dh = object_model.get_articulation("cockpit_door_hinge")
        ctx.check("cockpit_door_revolute", dh.articulation_type == ArticulationType.REVOLUTE and abs(dh.axis[2]) > 0.99, "cockpit door must be a vertical revolute")
        ctx.allow_overlap(door, fus, elem_a="door_skin", elem_b="hull", reason="Closed crew door sits flush in the flank opening.")
        ctx.allow_overlap(door, fus, elem_a="door_skin", elem_b="door_frame", reason="Closed crew door seats inside the frame surround.")
        ctx.allow_overlap(door, fus, elem_a="hinge_knuckle", elem_b="door_frame", reason="Hinge knuckle seats on the frame; on-axis.")
        ctx.allow_overlap(door, fus, elem_a="hinge_knuckle", elem_b="hull", reason="Hinge knuckle embeds into the flank edge to carry the door; on-axis.")
    if r.doors_module in ("bulkhead_door", "both"):
        bdoor = object_model.get_part("bulkhead_door")
        bh = object_model.get_articulation("bulkhead_door_hinge")
        ctx.check("bulkhead_door_revolute", bh.articulation_type == ArticulationType.REVOLUTE and abs(bh.axis[2]) > 0.99, "bulkhead door must be a vertical revolute")
        ctx.check("bulkhead_present", "cabin_bulkhead" in fus_vis, "bulkhead partition missing")
        ctx.allow_overlap(bdoor, fus, reason="Internal bulkhead door sits wholly inside the fuselage envelope; hinge knuckle nests in the jamb.")
        ctx.expect_within(bdoor, fus, axes=("x", "y", "z"), name="bulkhead door contained within the fuselage")

    return ctx.report()


HelicopterConfig = TandemRotorHelicopterConfig
ResolvedHelicopterConfig = ResolvedTandemRotorHelicopterConfig

__all__ = [
    "TandemRotorHelicopterConfig",
    "ResolvedTandemRotorHelicopterConfig",
    "HelicopterConfig",
    "ResolvedHelicopterConfig",
    "config_from_seed",
    "resolve_config",
    "build_tandem_rotor_helicopter",
    "build_seeded_tandem_rotor_helicopter",
    "slot_choices_for_seed",
    "run_tandem_rotor_helicopter_tests",
]
