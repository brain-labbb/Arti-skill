"""Modular hand-grenade template.

Slot graph (pattern: mixed — body-centric, not a serial chain):

    body_form (Slot A, root, grounded z=0)
        ├─ body_surface (Slot B): overlay fused into / parented onto the body
        │     └─ multiplicity: fragmentation rows (frag_knob_grid only)
        └─ fuze (Slot C): collar/housing visuals FIXED into body +
              lever REVOLUTE, pin(s) PRISMATIC, ring/bail REVOLUTE

Two incompatible body spines gate the surface and fuze candidates:

  * **ovoid_frag / tapered_ovoid** — `cq.Workplane("XZ")` lathe revolve of an
    ovoid/teardrop profile; mk2-family fuze (single cotter pin + torus ring) at
    the neck. Adapted verbatim from S1 (mk2 pineapple) / S3 (tapered egg) /
    S4 (smooth ovoid) / S7 (perforated vent ovoid).
  * **cylindrical_tube** — straight tube with hex end caps; m84-family fuze
    (twin bent-wire pins + rings) above the top cap. Adapted from S2 (m84
    perforated) / S5 (smooth cyl) / S6 (frag knobs on cyl).

The bail fuze (S8 single triangular bail, child of body) is the only fuze
compatible with every spine.

Invariant motion contract held in every seed:
  lever REVOLUTE axis -Y 0..~1.571 / pin(s) PRISMATIC axis +Y 0..0.020 /
  ring REVOLUTE axis +Y span ~2.1 (child of pin) OR bail REVOLUTE axis -X
  0..2.8 (child of body).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Literal

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    min_clearance_at_pose,
    tube_from_spline_points,
)

__modular__ = True

# ----------------------------------------------------------------------------
# Enum domains
# ----------------------------------------------------------------------------
BodyForm = Literal["ovoid_frag", "cylindrical_tube", "tapered_ovoid"]
BodySurface = Literal["frag_knob_grid", "smooth_shell", "perforated_vent_shell"]
Fuze = Literal["mk2_single_pin_ring", "m84_twin_pin_ring", "single_pin_bail"]
PaletteStyle = Literal[
    "olive_drab",
    "flat_black",
    "desert_sand",
    "od_with_ochre_band",
    "weathered_silver_fuze",
]

_LATHE_SPINES = ("ovoid_frag", "tapered_ovoid")
_FRAG_N_MIN, _FRAG_N_MAX = 3, 7

# Palette table: olive/black/sand recolors of the body cast + accents.  Every
# entry only swaps rgba (no topology change).  Keys are stable material roles.
_PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "olive_drab": {
        "body": (0.27, 0.32, 0.20, 1.0),
        "cap": (0.24, 0.28, 0.17, 1.0),
        "band": (0.82, 0.56, 0.10, 1.0),
        "fuze": (0.74, 0.75, 0.78, 1.0),
        "lever": (0.15, 0.19, 0.12, 1.0),
        "steel": (0.80, 0.81, 0.84, 1.0),
        "charge": (0.76, 0.65, 0.48, 1.0),
        "accent": (0.62, 0.18, 0.12, 1.0),
    },
    "flat_black": {
        "body": (0.10, 0.10, 0.11, 1.0),
        "cap": (0.07, 0.07, 0.08, 1.0),
        "band": (0.55, 0.42, 0.10, 1.0),
        "fuze": (0.30, 0.31, 0.33, 1.0),
        "lever": (0.06, 0.06, 0.07, 1.0),
        "steel": (0.72, 0.73, 0.76, 1.0),
        "charge": (0.30, 0.27, 0.22, 1.0),
        "accent": (0.55, 0.16, 0.11, 1.0),
    },
    "desert_sand": {
        "body": (0.74, 0.66, 0.48, 1.0),
        "cap": (0.66, 0.58, 0.41, 1.0),
        "band": (0.45, 0.33, 0.16, 1.0),
        "fuze": (0.70, 0.70, 0.71, 1.0),
        "lever": (0.55, 0.49, 0.35, 1.0),
        "steel": (0.80, 0.80, 0.82, 1.0),
        "charge": (0.58, 0.50, 0.36, 1.0),
        "accent": (0.58, 0.20, 0.13, 1.0),
    },
    "od_with_ochre_band": {
        "body": (0.30, 0.34, 0.21, 1.0),
        "cap": (0.26, 0.30, 0.18, 1.0),
        "band": (0.86, 0.62, 0.08, 1.0),
        "fuze": (0.70, 0.71, 0.73, 1.0),
        "lever": (0.16, 0.20, 0.12, 1.0),
        "steel": (0.80, 0.81, 0.84, 1.0),
        "charge": (0.78, 0.66, 0.48, 1.0),
        "accent": (0.64, 0.19, 0.12, 1.0),
    },
    "weathered_silver_fuze": {
        "body": (0.31, 0.35, 0.22, 1.0),
        "cap": (0.27, 0.31, 0.19, 1.0),
        "band": (0.40, 0.41, 0.43, 1.0),
        "fuze": (0.60, 0.61, 0.63, 1.0),
        "lever": (0.18, 0.21, 0.14, 1.0),
        "steel": (0.66, 0.67, 0.70, 1.0),
        "charge": (0.74, 0.64, 0.47, 1.0),
        "accent": (0.60, 0.18, 0.12, 1.0),
    },
}


# ============================================================================
# Config
# ============================================================================
@dataclass(frozen=True)
class GrenadeConfig:
    body_form: BodyForm = "ovoid_frag"
    body_surface: BodySurface = "frag_knob_grid"
    fuze: Fuze = "mk2_single_pin_ring"
    frag_row_count: int = 5
    palette_style: PaletteStyle = "olive_drab"
    body_height_scale: float = 1.0
    body_dia_scale: float = 1.0
    name: str = "reference_grenade"


@dataclass(frozen=True)
class ResolvedGrenadeConfig:
    body_form: BodyForm
    body_surface: BodySurface
    fuze: Fuze
    frag_row_count: int
    knob_height_scale: float
    palette_style: PaletteStyle
    body_height_scale: float
    body_dia_scale: float
    is_lathe: bool
    name: str


# ============================================================================
# Compatibility matrix + procedural sampling
# ============================================================================
def _legal_surfaces(body_form: BodyForm) -> tuple[BodySurface, ...]:
    # frag_knob_grid + smooth_shell are portable across every spine (S1/S4/S5/S6).
    # perforated_vent_shell needs a wall thick enough for a hollow cavity + radial
    # bores; the tapered teardrop base (r=0.006) is too narrow to host it, so it is
    # gated to the round ovoid (S7) and the hex tube (S2) spines only.
    if body_form == "tapered_ovoid":
        return ("frag_knob_grid", "smooth_shell")
    return ("frag_knob_grid", "smooth_shell", "perforated_vent_shell")


def _legal_fuzes(body_form: BodyForm) -> tuple[Fuze, ...]:
    if body_form == "cylindrical_tube":
        # m84 twin fuze only on the hex tube; bail also legal on every spine.
        return ("m84_twin_pin_ring", "single_pin_bail")
    # lathe spines: mk2 single-pin fuze + bail.
    return ("mk2_single_pin_ring", "single_pin_bail")


def config_from_seed(seed: int) -> GrenadeConfig:
    rng = random.Random(seed)

    body_form: BodyForm = rng.choices(
        ("ovoid_frag", "cylindrical_tube", "tapered_ovoid"),
        weights=(0.40, 0.34, 0.26),
        k=1,
    )[0]

    surface_weights = {
        "frag_knob_grid": 0.42,
        "smooth_shell": 0.28,
        "perforated_vent_shell": 0.30,
    }
    legal_surfaces = _legal_surfaces(body_form)
    body_surface: BodySurface = rng.choices(
        legal_surfaces,
        weights=[surface_weights[s] for s in legal_surfaces],
        k=1,
    )[0]

    fuze: Fuze = rng.choice(_legal_fuzes(body_form))

    # Fragmentation rows: only meaningful under frag_knob_grid; small-N biased.
    frag_row_count = 5
    if body_surface == "frag_knob_grid":
        frag_row_count = rng.choices(
            (3, 4, 5, 6, 7),
            weights=(0.26, 0.26, 0.24, 0.14, 0.10),
            k=1,
        )[0]

    palette_style: PaletteStyle = rng.choice(
        (
            "olive_drab",
            "flat_black",
            "desert_sand",
            "od_with_ochre_band",
            "weathered_silver_fuze",
        )
    )

    body_height_scale = round(rng.uniform(0.92, 1.08), 4)
    body_dia_scale = round(rng.uniform(0.92, 1.08), 4)

    return GrenadeConfig(
        body_form=body_form,
        body_surface=body_surface,
        fuze=fuze,
        frag_row_count=frag_row_count,
        palette_style=palette_style,
        body_height_scale=body_height_scale,
        body_dia_scale=body_dia_scale,
        name=f"seeded_grenade_{seed}",
    )


def resolve_config(config: GrenadeConfig) -> ResolvedGrenadeConfig:
    if config.body_form not in ("ovoid_frag", "cylindrical_tube", "tapered_ovoid"):
        raise ValueError(f"Unsupported body_form: {config.body_form}")
    if config.body_surface not in (
        "frag_knob_grid",
        "smooth_shell",
        "perforated_vent_shell",
    ):
        raise ValueError(f"Unsupported body_surface: {config.body_surface}")
    if config.fuze not in ("mk2_single_pin_ring", "m84_twin_pin_ring", "single_pin_bail"):
        raise ValueError(f"Unsupported fuze: {config.fuze}")
    if config.palette_style not in _PALETTES:
        raise ValueError(f"Unsupported palette_style: {config.palette_style}")

    # Gate the surface by spine (vent shell off the narrow-base taper).
    legal_surfaces = _legal_surfaces(config.body_form)
    body_surface = (
        config.body_surface if config.body_surface in legal_surfaces else legal_surfaces[0]
    )

    # Gate the fuze by spine (degrade an illegal combo rather than crash builder).
    legal_fuzes = _legal_fuzes(config.body_form)
    fuze = config.fuze if config.fuze in legal_fuzes else legal_fuzes[0]

    # frag_row_count only lives under frag_knob_grid; otherwise it is inert.
    frag_row_count = 5
    knob_height_scale = 1.0
    if body_surface == "frag_knob_grid":
        frag_row_count = max(_FRAG_N_MIN, min(_FRAG_N_MAX, int(config.frag_row_count)))
        # denser rows -> shorter knobs (matches frag7row reducing KNOB_H).
        knob_height_scale = max(0.70, min(1.10, (5.0 / frag_row_count) ** 0.5))

    body_height_scale = max(0.92, min(1.08, float(config.body_height_scale)))
    body_dia_scale = max(0.92, min(1.08, float(config.body_dia_scale)))

    return ResolvedGrenadeConfig(
        body_form=config.body_form,
        body_surface=body_surface,
        fuze=fuze,
        frag_row_count=frag_row_count,
        knob_height_scale=knob_height_scale,
        palette_style=config.palette_style,
        body_height_scale=body_height_scale,
        body_dia_scale=body_dia_scale,
        is_lathe=config.body_form in _LATHE_SPINES,
        name=config.name,
    )


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    r = resolve_config(config_from_seed(seed))
    surface_mod = r.body_surface
    if r.body_surface == "frag_knob_grid":
        surface_mod = f"frag_{r.frag_row_count}row_grid"
    return [
        ("body_form", r.body_form),
        ("body_surface", surface_mod),
        ("fuze", r.fuze),
    ]


# ============================================================================
# Shared lathe helpers (mk2 / tapered ovoid family — S1 / S3 / S4 / S7)
# ============================================================================
# Outer (full) body lathe profile: (radius, z), base flat at z=0 (S1 L56-L68).
_OVOID_BODY_PROFILE = [
    (0.0140, 0.000),
    (0.0240, 0.006),
    (0.0282, 0.016),
    (0.0298, 0.028),
    (0.0300, 0.040),
    (0.0285, 0.052),
    (0.0252, 0.064),
    (0.0205, 0.072),
    (0.0160, 0.079),
    (0.0132, 0.0825),
    (0.0130, 0.085),
]
# Recessed waffle-groove core profile (S1 L71-L85).
_OVOID_CORE_PROFILE = [
    (0.0140, 0.000),
    (0.0200, 0.004),
    (0.0235, 0.012),
    (0.0250, 0.022),
    (0.0262, 0.034),
    (0.0263, 0.044),
    (0.0248, 0.054),
    (0.0215, 0.064),
    (0.0180, 0.071),
    (0.0160, 0.0765),
    (0.0140, 0.081),
    (0.0130, 0.0835),
    (0.0130, 0.085),
]
# Tapered teardrop egg profile (S3 L57-L71).
_TAPER_BODY_PROFILE = [
    (0.0060, 0.000),
    (0.0085, 0.005),
    (0.0110, 0.012),
    (0.0145, 0.022),
    (0.0185, 0.033),
    (0.0230, 0.044),
    (0.0270, 0.054),
    (0.0300, 0.062),
    (0.0295, 0.068),
    (0.0260, 0.074),
    (0.0210, 0.078),
    (0.0160, 0.082),
    (0.0130, 0.085),
]
_TAPER_CORE_PROFILE = [
    (0.0060, 0.000),
    (0.0070, 0.004),
    (0.0090, 0.010),
    (0.0115, 0.018),
    (0.0150, 0.028),
    (0.0190, 0.039),
    (0.0230, 0.049),
    (0.0260, 0.058),
    (0.0270, 0.063),
    (0.0255, 0.070),
    (0.0220, 0.075),
    (0.0175, 0.080),
    (0.0140, 0.083),
    (0.0130, 0.085),
]

_OVOID_BODY_TOP_Z = 0.085

# mk2 fuze geometry constants (S1 L37-L53).
_MK2_COLLAR_Z0, _MK2_COLLAR_Z1 = 0.0870, 0.0950
_MK2_COLLAR_R = 0.0105
_MK2_HOUSING_HX, _MK2_HOUSING_HY = 0.007, 0.004
_MK2_HOUSING_Z0, _MK2_HOUSING_Z1 = 0.0940, 0.1140
_MK2_PIN_Z = 0.1035
_MK2_PIN_HOLE_R = 0.0024
_MK2_PIN_SHAFT_R = 0.00245
_MK2_AXLE_R = 0.00175
_MK2_EAR_HOLE_R = 0.0017
_MK2_LEVER_W_HALF = 0.0058
_MK2_LEVER_T_HALF = 0.0008
_MK2_LEVER_PIVOT = (-0.0078, 0.1126)
_MK2_RING_R = 0.0130
_MK2_RING_TUBE_R = 0.0021
_MK2_RING_JOINT_Y = 0.00825
_PIN_TRAVEL = 0.020

_MK2_LEVER_PTS = [
    (-0.0108, 0.1126),
    (-0.0102, 0.1150),
    (0.0080, 0.1150),
    (0.0110, 0.1070),
    (0.0165, 0.0940),
    (0.0205, 0.0845),
    (0.0235, 0.0770),
    (0.0285, 0.0680),
    (0.0312, 0.0585),
    (0.0330, 0.0485),
    (0.0335, 0.0405),
]
_TAPER_LEVER_PTS = [
    (-0.0108, 0.1126),
    (-0.0102, 0.1150),
    (0.0080, 0.1150),
    (0.0093, 0.1070),
    (0.0120, 0.0940),
    (0.0175, 0.0845),
    (0.0270, 0.0770),
    (0.0340, 0.0680),
    (0.0330, 0.0580),
    (0.0300, 0.0480),
    (0.0260, 0.0400),
]

# frag knob grid (ovoid) constants (S1 L88-L93).
_KNOB_COLS = 8
_KNOB_COL_OFFSET_DEG = 22.5
_KNOB_H = 0.0111
_KNOB_T = 0.0060
_KNOB_GAP = 0.0040
_KNOB_CHAMFER = 0.0012

# perforated vent (ovoid) constants (S7).
_VENT_SHELL_WALL = 0.003
_VENT_CAVITY_FLOOR_Z = 0.006
_VENT_CAVITY_CEILING_Z = 0.080
_VENT_HOLE_COLS = 10
_VENT_HOLE_COL_OFFSET_DEG = 18.0
_VENT_HOLE_R = 0.003
_VENT_HOLE_DEPTH = 0.008
_VENT_CHARGE_TUBE_R = 0.010
_VENT_CHARGE_TUBE_Z0 = 0.008
_VENT_CHARGE_TUBE_Z1 = 0.076
_VENT_CHARGE_RING_Z = [0.025, 0.055]
_VENT_CHARGE_RING_H = 0.0015


def _profiles_for(r: ResolvedGrenadeConfig) -> tuple[list, list]:
    if r.body_form == "tapered_ovoid":
        return _TAPER_BODY_PROFILE, _TAPER_CORE_PROFILE
    return _OVOID_BODY_PROFILE, _OVOID_CORE_PROFILE


def _lever_pts_for(r: ResolvedGrenadeConfig) -> list[tuple[float, float]]:
    return _TAPER_LEVER_PTS if r.body_form == "tapered_ovoid" else _MK2_LEVER_PTS


def _interp_radius(profile: list[tuple[float, float]], z: float) -> float:
    if z <= profile[0][1]:
        return profile[0][0]
    for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
        if z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + t * (r1 - r0)
    return profile[-1][0]


def _surface_normal(profile: list[tuple[float, float]], z: float) -> tuple[float, float]:
    eps = 0.002
    dr = _interp_radius(profile, z + eps) - _interp_radius(profile, z - eps)
    dz = 2.0 * eps
    n = (dz, -dr)
    mag = math.hypot(*n)
    return (n[0] / mag, n[1] / mag)


def _lathe(profile: list[tuple[float, float]]) -> cq.Workplane:
    wp = cq.Workplane("XZ").moveTo(0.0, 0.0)
    for rr, zz in profile:
        wp = wp.lineTo(rr, zz)
    wp = wp.lineTo(0.0, profile[-1][1]).close()
    return wp.revolve(360.0, (0.0, 0.0), (0.0, 1.0))


def _strip_polygon(pts: list[tuple[float, float]], half_t: float) -> list[tuple[float, float]]:
    seg_n: list[tuple[float, float]] = []
    for (x0, z0), (x1, z1) in zip(pts, pts[1:]):
        dx, dz = x1 - x0, z1 - z0
        mag = math.hypot(dx, dz)
        seg_n.append((-dz / mag, dx / mag))
    normals: list[tuple[float, float]] = []
    for i in range(len(pts)):
        if i == 0:
            n = seg_n[0]
        elif i == len(pts) - 1:
            n = seg_n[-1]
        else:
            nx = seg_n[i - 1][0] + seg_n[i][0]
            nz = seg_n[i - 1][1] + seg_n[i][1]
            mag = math.hypot(nx, nz) or 1.0
            n = (nx / mag, nz / mag)
        normals.append(n)
    outer = [(p[0] + n[0] * half_t, p[1] + n[1] * half_t) for p, n in zip(pts, normals)]
    inner = [(p[0] - n[0] * half_t, p[1] - n[1] * half_t) for p, n in zip(pts, normals)]
    return outer + inner[::-1]


def _frag_knob_rows_z(profile: list[tuple[float, float]], n_rows: int) -> list[float]:
    """Equal-z rows spread over the body flank (S1/S9/S10 KNOB_ROWS_Z)."""
    z_lo, z_hi = 0.012, 0.072
    if n_rows == 1:
        return [0.5 * (z_lo + z_hi)]
    step = (z_hi - z_lo) / (n_rows - 1)
    return [z_lo + step * i for i in range(n_rows)]


def _ovoid_radial_hole_cutter(profile, z_c: float, col_index: int) -> cq.Solid:
    r_surface = _interp_radius(profile, z_c)
    theta = math.radians(360.0 * col_index / _VENT_HOLE_COLS + _VENT_HOLE_COL_OFFSET_DEG)
    start_r = r_surface + 0.002
    cx = start_r * math.cos(theta)
    cy = start_r * math.sin(theta)
    return cq.Solid.makeCylinder(
        _VENT_HOLE_R,
        _VENT_HOLE_DEPTH,
        cq.Vector(cx, cy, z_c),
        cq.Vector(-math.cos(theta), -math.sin(theta), 0.0),
    )


def _mk2_fuze_shape() -> cq.Compound:
    """Threaded collar + pivot housing + lug + axle (S1 _fuze_shape)."""
    collar = cq.Solid.makeCylinder(
        _MK2_COLLAR_R,
        _MK2_COLLAR_Z1 - _MK2_COLLAR_Z0,
        cq.Vector(0, 0, _MK2_COLLAR_Z0),
        cq.Vector(0, 0, 1),
    )
    housing = (
        cq.Workplane("XY")
        .box(2 * _MK2_HOUSING_HX, 2 * _MK2_HOUSING_HY, _MK2_HOUSING_Z1 - _MK2_HOUSING_Z0)
        .edges(">Z")
        .chamfer(0.0008)
        .translate((0.0, 0.0, (_MK2_HOUSING_Z0 + _MK2_HOUSING_Z1) / 2.0))
    )
    pin_hole = cq.Solid.makeCylinder(
        _MK2_PIN_HOLE_R, 0.04, cq.Vector(0.0, -0.02, _MK2_PIN_Z), cq.Vector(0, 1, 0)
    )
    housing = housing.cut(pin_hole)
    lug = (
        cq.Workplane("XY")
        .box(0.0040, 2 * _MK2_HOUSING_HY, 0.0040)
        .translate((-0.00765, 0.0, 0.1126))
    )
    axle = cq.Solid.makeCylinder(
        _MK2_AXLE_R,
        0.0136,
        cq.Vector(_MK2_LEVER_PIVOT[0], -0.0068, _MK2_LEVER_PIVOT[1]),
        cq.Vector(0, 1, 0),
    )
    return cq.Compound.makeCompound([collar, housing.val(), lug.val(), axle])


def _mk2_lever_shape(
    lever_pts: list[tuple[float, float]],
    dia_scale: float = 1.0,
    height_scale: float = 1.0,
) -> cq.Compound:
    """Curved spoon strip + two pivot ear plates (S1 _lever_shape).

    The polyline is expressed pivot-locally; when the body is scaled up the
    spoon reach is scaled radially (x) / vertically (z) about the pivot so the
    spoon keeps its 1-3 mm running clearance over the scaled flank (spec lever
    clearance inequality — re-offset polyline outward when scaled).
    """
    px, pz = _MK2_LEVER_PIVOT
    local_pts = [((x - px) * dia_scale, (z - pz) * height_scale) for x, z in lever_pts]
    poly = _strip_polygon(local_pts, _MK2_LEVER_T_HALF)
    wp = cq.Workplane("XZ").moveTo(*poly[0])
    for p in poly[1:]:
        wp = wp.lineTo(*p)
    strip = wp.close().extrude(_MK2_LEVER_W_HALF, both=True)
    ears = []
    for sign in (1.0, -1.0):
        y_c = sign * 0.0052
        plate = cq.Workplane("XY").box(0.008, 0.0012, 0.0060).translate((0.0, y_c, 0.0))
        hole = cq.Solid.makeCylinder(
            _MK2_EAR_HOLE_R, 0.002, cq.Vector(0.0, y_c - 0.001, 0.0), cq.Vector(0, 1, 0)
        )
        ears.append(plate.cut(hole).val())
    return cq.Compound.makeCompound([strip.val()] + ears)


def _mk2_pin_shape() -> cq.Compound:
    """Cotter-style pull pin: shaft + legs + head disc (S1 _pin_shape)."""
    shaft = cq.Solid.makeCylinder(
        _MK2_PIN_SHAFT_R, 0.0130, cq.Vector(0.0, -0.0030, 0.0), cq.Vector(0, 1, 0)
    )
    legs = [
        cq.Solid.makeCylinder(
            0.0010, 0.0095, cq.Vector(0.0, -0.0115, sz * 0.0011), cq.Vector(0, 1, 0)
        )
        for sz in (1.0, -1.0)
    ]
    head = cq.Solid.makeCylinder(0.0035, 0.0035, cq.Vector(0.0, 0.0065, 0.0), cq.Vector(0, 1, 0))
    return cq.Compound.makeCompound([shaft, head] + legs)


def _mk2_ring_shape() -> cq.Solid:
    """Pull ring torus threaded through the pin-head eye (S1 _ring_shape)."""
    off = _MK2_RING_R / math.sqrt(2.0)
    return cq.Solid.makeTorus(
        _MK2_RING_R, _MK2_RING_TUBE_R, cq.Vector(off, 0.0, off), cq.Vector(0, 1, 0)
    )


# ============================================================================
# Cylinder (m84) spine helpers — S2 / S5 / S6 / S8
# ============================================================================
_CYL_BODY_R_OUT = 0.024
_CYL_BODY_R_IN = 0.0195
_CYL_TUBE_R = 0.0193
_CYL_HEX_R = 0.028
_CYL_CAP_BOT_Z0, _CYL_CAP_BOT_Z1 = 0.0, 0.020
_CYL_SHELL_Z0, _CYL_SHELL_Z1 = 0.019, 0.091
_CYL_CAP_TOP_Z0, _CYL_CAP_TOP_Z1 = 0.090, 0.112
_CYL_HOLE_R = 0.006
_CYL_ROW_Z = (0.0325, 0.055, 0.0775)
_CYL_HOLES_PER_ROW = 5
_CYL_BAND_R_IN, _CYL_BAND_R_OUT = 0.0238, 0.0252
_CYL_BAND_Z0, _CYL_BAND_Z1 = 0.046, 0.064
_CYL_FUZE_COLLAR_R, _CYL_FUZE_COLLAR_Z0, _CYL_FUZE_COLLAR_Z1 = 0.013, 0.112, 0.1155
_CYL_FUZE_BODY_R, _CYL_FUZE_BODY_Z0, _CYL_FUZE_BODY_Z1 = 0.0115, 0.1155, 0.1265
_CYL_LEVER_PIVOT = (0.010, 0.0, 0.128)
_CYL_PIN_SHAFT_R = 0.0016
_CYL_PIN1_Z = 0.1215
_CYL_PIN2_Z = 0.1175
_CYL_PIN2_YAW = math.radians(75.0)
_CYL_PIN_MOUNT_R = 0.0135
_CYL_RING_MAJOR_R = 0.016
_CYL_RING_WIRE_R = 0.0018
_CYL_BAIL_PIVOT_Z = 0.122
_CYL_BAIL_WIRE_R = 0.0018
_CYL_BAIL_PIVOT_PIN_R = 0.0014
_CYL_BAIL_PIVOT_PIN_LEN = 0.028
# frag-on-cyl knob constants (S6).
_CYL_KNOB_BASE_R = 0.0025
_CYL_KNOB_TOP_R = 0.0015
_CYL_KNOB_H = 0.003
_CYL_KNOB_EMBED = 0.001
_CYL_KNOB_COLS = 8


def _hex_prism(height: float) -> cq.Workplane:
    pts = [
        (
            _CYL_HEX_R * math.cos(math.radians(30 + 60 * k)),
            _CYL_HEX_R * math.sin(math.radians(30 + 60 * k)),
        )
        for k in range(6)
    ]
    return cq.Workplane("XY").polyline(pts).close().extrude(height)


def _cyl_perforated_shell(sz: float = 1.0) -> cq.Workplane:
    h = (_CYL_SHELL_Z1 - _CYL_SHELL_Z0) * sz
    shell = cq.Workplane("XY").circle(_CYL_BODY_R_OUT).circle(_CYL_BODY_R_IN).extrude(h)
    for row_z in _CYL_ROW_Z:
        zl = (row_z - _CYL_SHELL_Z0) * sz
        for k in range(_CYL_HOLES_PER_ROW):
            a = 2.0 * math.pi * k / _CYL_HOLES_PER_ROW
            cutter = cq.Solid.makeCylinder(
                _CYL_HOLE_R,
                0.035,
                cq.Vector(0.0, 0.0, zl),
                cq.Vector(math.cos(a), math.sin(a), 0.0),
            )
            shell = shell.cut(cutter)
    return shell


def _cyl_solid_shell(sz: float = 1.0) -> cq.Workplane:
    h = (_CYL_SHELL_Z1 - _CYL_SHELL_Z0) * sz
    return cq.Workplane("XY").circle(_CYL_BODY_R_OUT).circle(_CYL_BODY_R_IN).extrude(h)


def _cyl_mid_band(with_holes: bool, sz: float = 1.0) -> cq.Workplane:
    h = (_CYL_BAND_Z1 - _CYL_BAND_Z0) * sz
    band = cq.Workplane("XY").circle(_CYL_BAND_R_OUT).circle(_CYL_BAND_R_IN).extrude(h)
    if not with_holes:
        return band
    zl = (_CYL_ROW_Z[1] - _CYL_BAND_Z0) * sz
    for k in range(_CYL_HOLES_PER_ROW):
        a = 2.0 * math.pi * k / _CYL_HOLES_PER_ROW
        cutter = cq.Solid.makeCylinder(
            _CYL_HOLE_R + 0.0003,
            0.035,
            cq.Vector(0.0, 0.0, zl),
            cq.Vector(math.cos(a), math.sin(a), 0.0),
        )
        band = band.cut(cutter)
    return band


def _cyl_fragmentation_knob(r_base: float, r_top: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(r_base).workplane(offset=height).circle(r_top).loft()


def _cyl_knob_rows_z(n_rows: int) -> list[float]:
    shell_h = _CYL_SHELL_Z1 - _CYL_SHELL_Z0
    return [_CYL_SHELL_Z0 + shell_h * (i + 1) / (n_rows + 1) for i in range(n_rows)]


def _cyl_ring_mesh(name: str):
    pts = []
    n = 28
    for i in range(n):
        t = 2.0 * math.pi * i / n
        pts.append((_CYL_RING_MAJOR_R * math.sin(t), 0.0, 0.0155 - _CYL_RING_MAJOR_R * math.cos(t)))
    geo = tube_from_spline_points(
        pts, radius=_CYL_RING_WIRE_R, closed_spline=True, samples_per_segment=4, radial_segments=10
    )
    return mesh_from_geometry(geo, name)


def _cyl_bail_ring_mesh(name: str):
    """Triangular bent-wire bail loop in the local YZ plane (S8 _bail_ring_mesh)."""
    verts = [(0.0, 0.0, 0.0), (0.0, -0.033, -0.0175), (0.0, -0.033, 0.0175)]
    pts: list[tuple[float, float, float]] = []
    n_per_side = 6
    for i in range(3):
        v_a = verts[i]
        v_b = verts[(i + 1) % 3]
        for j in range(n_per_side):
            t = j / n_per_side
            pts.append(
                (
                    v_a[0] + t * (v_b[0] - v_a[0]),
                    v_a[1] + t * (v_b[1] - v_a[1]),
                    v_a[2] + t * (v_b[2] - v_a[2]),
                )
            )
    geo = tube_from_spline_points(
        pts, radius=_CYL_BAIL_WIRE_R, closed_spline=True, samples_per_segment=4, radial_segments=10
    )
    return mesh_from_geometry(geo, name)


def _cyl_pin_visuals(pin, steel) -> None:
    pin.visual(
        Cylinder(radius=_CYL_PIN_SHAFT_R, length=0.029),
        origin=Origin(xyz=(0.0, -0.0145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shaft",
    )
    pin.visual(
        Cylinder(radius=0.0033, length=0.003),
        origin=Origin(xyz=(0.0, 0.0005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="eye_head",
    )


# m84 pull-ring swing-clearance geometry.  The ring loop is a torus whose centre
# sits _CYL_RING_MAJOR_R along the ring-local +z (pivoting about the pin +Y axis
# with a baked-in pre-tilt).  When swung the loop's lowest point dips toward the
# hex top cap; past a critical angle it plunges into the cap (the seed-5 穿模).
# Derive each ring's safe swing arc directly from the cap-top plane instead of a
# hand-guessed symmetric limit: keep the loop's lowest point >= CLR above the cap.
_M84_RING_CENTER_Z = 0.0155  # ring-local z of the torus centre (see _cyl_ring_mesh)
_M84_RING_CAP_CLR = 0.0025  # standoff kept between the loop and the cap top face


def _m84_ring_swing_limits(
    sz: float,
    pin_native_z: float,
    prerot: float,
    orig_lower: float,
    orig_upper: float,
    clr: float = _M84_RING_CAP_CLR,
) -> tuple[float, float]:
    """Swing bounds keeping the pull-ring loop clear of the scaled hex top cap.

    ``prerot`` is the ring visual's baked Y pre-tilt; the joint swing adds to it,
    so the total tilt is ``prerot + swing``.  The loop's lowest world z is
    ``pivot_z + _M84_RING_CENTER_Z*cos(phi) - _CYL_RING_MAJOR_R``; require it to
    clear ``cap_top + clr`` and solve the symmetric ``|phi| <= phi_max`` window.
    """
    zoff = _cyl_fuze_zoff(sz)
    pivot_z = pin_native_z + zoff
    cap_top = _CYL_CAP_TOP_Z1 * sz
    rhs = (cap_top + clr - pivot_z + _CYL_RING_MAJOR_R) / _M84_RING_CENTER_Z
    rhs = max(-1.0, min(1.0, rhs))
    phi_max = math.acos(rhs)
    lower = max(orig_lower, -(phi_max + prerot))
    upper = min(orig_upper, phi_max - prerot)
    # Keep the rest pose (swing=0) inside the solved window.
    return (min(lower, 0.0), max(upper, 0.0))


# ============================================================================
# Body + surface builders (Slot A + Slot B)
# ============================================================================
def _build_lathe_body(model: ArticulatedObject, r: ResolvedGrenadeConfig, mats: dict) -> object:
    """Ovoid / tapered lathe spine + its surface overlay, all fused into one mesh.

    Returns the root body part.  Lathe spines fuse the surface into the
    `grenade_body` Compound mesh (S1/S7).  We scale the whole mesh via a
    visual Origin scale baked into vertex coordinates is not supported, so we
    scale the cadquery profiles up-front by editing the dia/height factors.
    """
    profile, core = _profiles_for(r)
    sx, sz = r.body_dia_scale, r.body_height_scale

    def scl(prof):
        return [(rr * sx, zz * sz) for rr, zz in prof]

    sprofile, score = scl(profile), scl(core)

    body = model.part("grenade_body")

    if r.body_surface == "frag_knob_grid":
        shape = _ovoid_frag_body_shape_scaled(r, sprofile, score)
        body.visual(
            mesh_from_cadquery(shape, "grenade_body", tolerance=0.0005),
            material=mats["body"],
            name="frag_body",
        )
    elif r.body_surface == "perforated_vent_shell":
        shell = _ovoid_perforated_body_shape_scaled(sprofile)
        body.visual(
            mesh_from_cadquery(shell, "perforated_shell", tolerance=0.0004),
            material=mats["body"],
            name="perforated_shell",
        )
        body.visual(
            mesh_from_cadquery(
                _ovoid_charge_tube_shape_scaled(sprofile), "charge_tube", tolerance=0.0004
            ),
            material=mats["charge"],
            name="charge_tube",
        )
    else:  # smooth_shell
        body.visual(
            mesh_from_cadquery(_lathe(sprofile), "grenade_body", tolerance=0.0005),
            material=mats["body"],
            name="smooth_body",
        )

    # Ochre/painted neck band decoration (parent visual, S1 L284-L289).
    body.visual(
        Cylinder(radius=0.0138 * sx, length=0.0070 * sz),
        origin=Origin(xyz=(0.0, 0.0, 0.0845 * sz)),
        material=mats["band"],
        name="neck_band",
    )
    return body


def _ovoid_frag_body_shape_scaled(r, sprofile, score) -> cq.Compound:
    """frag knobs fused onto a pre-scaled lathe core."""
    solids = [_lathe(score).val()]
    knob_h = _KNOB_H * r.knob_height_scale * r.body_height_scale
    knob_t = _KNOB_T * r.body_dia_scale
    for z_c in _frag_knob_rows_z(sprofile, r.frag_row_count):
        r_c = _interp_radius(sprofile, z_c)
        nx, nz = _surface_normal(sprofile, z_c)
        tilt_deg = math.degrees(math.atan2(-nz, nx))
        width = 2.0 * math.pi * r_c / _KNOB_COLS - _KNOB_GAP
        cx = r_c - 0.0025 * nx
        cz = z_c - 0.0025 * nz
        knob = cq.Workplane("XY").box(knob_t, width, knob_h).edges().chamfer(_KNOB_CHAMFER).val()
        knob = knob.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), tilt_deg)
        knob = knob.translate(cq.Vector(cx, 0.0, cz))
        for k in range(_KNOB_COLS):
            theta = _KNOB_COL_OFFSET_DEG + 45.0 * k
            solids.append(knob.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), theta))
    return cq.Compound.makeCompound(solids)


def _ovoid_perforated_body_shape_scaled(sprofile) -> cq.Compound:
    outer = _lathe(sprofile).val()
    floor_z = _VENT_CAVITY_FLOOR_Z
    ceil_z = _VENT_CAVITY_CEILING_Z * (sprofile[-1][1] / 0.085)
    cavity_pts = [(0.0, floor_z)]
    for rr, zz in sprofile:
        if zz < floor_z:
            continue
        if zz > ceil_z:
            break
        ir = max(rr - _VENT_SHELL_WALL, 0.004)
        cavity_pts.append((ir, zz))
    cavity_pts.append((0.0, ceil_z))
    wp = cq.Workplane("XZ").moveTo(*cavity_pts[0])
    for p in cavity_pts[1:]:
        wp = wp.lineTo(*p)
    cavity = wp.close().revolve(360.0, (0.0, 0.0), (0.0, 1.0)).val()
    shell = outer.cut(cavity)
    holes: list[cq.Solid] = []
    for z_c in _frag_knob_rows_z(sprofile, 5):
        for j in range(_VENT_HOLE_COLS):
            holes.append(_ovoid_radial_hole_cutter(sprofile, z_c, j))
    return shell.cut(cq.Compound.makeCompound(holes))


def _ovoid_charge_tube_shape_scaled(sprofile) -> cq.Compound:
    z_scale = sprofile[-1][1] / 0.085
    tube_z0 = _VENT_CHARGE_TUBE_Z0 * z_scale
    tube_z1 = _VENT_CHARGE_TUBE_Z1 * z_scale
    solids: list[cq.Shape] = [
        cq.Solid.makeCylinder(
            _VENT_CHARGE_TUBE_R, tube_z1 - tube_z0, cq.Vector(0.0, 0.0, tube_z0), cq.Vector(0, 0, 1)
        )
    ]
    for z_ring0 in _VENT_CHARGE_RING_Z:
        z_ring = z_ring0 * z_scale
        r_shell_inner = _interp_radius(sprofile, z_ring) - _VENT_SHELL_WALL
        r_out = max(r_shell_inner + 0.0005, _VENT_CHARGE_TUBE_R + 0.001)
        r_in = _VENT_CHARGE_TUBE_R - 0.0005
        outer_cyl = cq.Solid.makeCylinder(
            r_out,
            _VENT_CHARGE_RING_H,
            cq.Vector(0.0, 0.0, z_ring - _VENT_CHARGE_RING_H / 2.0),
            cq.Vector(0, 0, 1),
        )
        bore = cq.Solid.makeCylinder(
            r_in,
            _VENT_CHARGE_RING_H + 0.001,
            cq.Vector(0.0, 0.0, z_ring - _VENT_CHARGE_RING_H / 2.0 - 0.0005),
            cq.Vector(0, 0, 1),
        )
        solids.append(outer_cyl.cut(bore))
    return cq.Compound.makeCompound(solids)


def _cyl_top_cap_z0(sz: float) -> float:
    """Bottom of the top hex cap (the whole cyl stack scales by sz from z=0)."""
    return _CYL_CAP_TOP_Z0 * sz


def _cyl_fuze_zoff(sz: float) -> float:
    """Z offset that seats the fuze stack on the (uniformly scaled) top cap.

    Every cyl z (cap heights, shell, band) scales by sz from z=0, so the fuze
    block — which is built at its native z — simply shifts by (sz-1)*collar_z0
    to land on the scaled top-cap top while staying contiguous."""
    return (sz - 1.0) * _CYL_FUZE_COLLAR_Z0


def _build_cyl_body(model: ArticulatedObject, r: ResolvedGrenadeConfig, mats: dict) -> object:
    """Hex-cap cylinder spine + its surface overlay, emitted as `body` visuals."""
    sz = r.body_height_scale
    body = model.part("body")

    body.visual(
        mesh_from_cadquery(_hex_prism((_CYL_CAP_BOT_Z1 - _CYL_CAP_BOT_Z0) * sz), "hex_cap_bottom"),
        origin=Origin(xyz=(0.0, 0.0, _CYL_CAP_BOT_Z0 * sz)),
        material=mats["cap"],
        name="hex_cap_bottom",
    )

    vent = r.body_surface == "perforated_vent_shell"
    body.visual(
        mesh_from_cadquery(
            _cyl_perforated_shell(sz) if vent else _cyl_solid_shell(sz),
            "perforated_shell",
            tolerance=0.0003,
        ),
        origin=Origin(xyz=(0.0, 0.0, _CYL_SHELL_Z0 * sz)),
        material=mats["body"],
        name="perforated_shell",
    )

    body.visual(
        Cylinder(radius=_CYL_TUBE_R, length=(_CYL_SHELL_Z1 - _CYL_SHELL_Z0) * sz),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (_CYL_SHELL_Z0 + _CYL_SHELL_Z1) * sz)),
        material=mats["charge"],
        name="charge_tube",
    )
    body.visual(
        mesh_from_cadquery(_cyl_mid_band(vent, sz), "mid_band", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, _CYL_BAND_Z0 * sz)),
        material=mats["band"],
        name="mid_band",
    )

    if r.body_surface == "perforated_vent_shell":
        body.visual(
            Cylinder(radius=0.0052, length=0.0038),
            origin=Origin(xyz=(0.0205, 0.0, _CYL_ROW_Z[0] * sz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=mats["accent"],
            name="flash_insert",
        )

    if r.body_surface == "frag_knob_grid":
        knob_h = _CYL_KNOB_H * r.knob_height_scale
        for i_row, row_z in enumerate(_cyl_knob_rows_z(r.frag_row_count)):
            for i_col in range(_CYL_KNOB_COLS):
                theta = 2.0 * math.pi * i_col / _CYL_KNOB_COLS
                body.visual(
                    mesh_from_cadquery(
                        _cyl_fragmentation_knob(_CYL_KNOB_BASE_R, _CYL_KNOB_TOP_R, knob_h),
                        f"frag_knob_{i_row}_{i_col}",
                    ),
                    origin=Origin(
                        xyz=(
                            (_CYL_BODY_R_OUT - _CYL_KNOB_EMBED) * math.cos(theta),
                            (_CYL_BODY_R_OUT - _CYL_KNOB_EMBED) * math.sin(theta),
                            row_z * sz - 0.5 * knob_h,
                        ),
                        rpy=(0.0, math.pi / 2.0, theta),
                    ),
                    material=mats["body"],
                    name=f"frag_knob_{i_row}_{i_col}",
                )

    cap_z0 = _cyl_top_cap_z0(sz)
    cap_h = (_CYL_CAP_TOP_Z1 - _CYL_CAP_TOP_Z0) * sz
    body.visual(
        mesh_from_cadquery(_hex_prism(cap_h), "hex_cap_top"),
        origin=Origin(xyz=(0.0, 0.0, cap_z0)),
        material=mats["cap"],
        name="hex_cap_top",
    )
    apothem = _CYL_HEX_R * math.cos(math.radians(30.0))
    stencil_z = cap_z0 + 0.5 * cap_h  # mid-height of the top cap, proud on its flat
    for i in range(7):
        y = 0.0085 - i * (0.017 / 6.0)
        body.visual(
            Box((0.0008, 0.0016, 0.005)),
            origin=Origin(xyz=(-(apothem + 0.0002), y, stencil_z)),
            material=mats["steel"],
            name=f"stencil_char_{i}",
        )
    return body


# ============================================================================
# Fuze builders (Slot C)
# ============================================================================
def _build_mk2_fuze(model: ArticulatedObject, body, r: ResolvedGrenadeConfig, mats: dict) -> None:
    """Single cotter pin + torus ring fuze on a lathe spine (S1)."""
    sz = r.body_height_scale
    # Fuze stack is on the central axis -> z scales, x/y stays (axisymmetric).
    body.visual(
        mesh_from_cadquery(_mk2_fuze_shape(), "fuze_assembly", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, (sz - 1.0) * _OVOID_BODY_TOP_Z)),
        material=mats["fuze"],
        name="fuze_assembly",
    )
    zoff = (sz - 1.0) * _OVOID_BODY_TOP_Z

    lever = model.part("safety_lever")
    lever.visual(
        mesh_from_cadquery(
            _mk2_lever_shape(_lever_pts_for(r), r.body_dia_scale, r.body_height_scale),
            "safety_lever",
            tolerance=0.0003,
        ),
        material=mats["lever"],
        name="spoon",
    )
    pin = model.part("safety_pin")
    pin.visual(
        mesh_from_cadquery(_mk2_pin_shape(), "safety_pin", tolerance=0.0003),
        material=mats["steel"],
        name="cotter_pin",
    )
    ring = model.part("pull_ring")
    ring.visual(
        mesh_from_cadquery(_mk2_ring_shape(), "pull_ring", tolerance=0.0003),
        material=mats["steel"],
        name="ring_torus",
    )

    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(_MK2_LEVER_PIVOT[0], 0.0, _MK2_LEVER_PIVOT[1] + zoff)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.571, effort=5.0, velocity=4.0),
    )
    model.articulation(
        "pin_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pin,
        origin=Origin(xyz=(0.0, 0.0, _MK2_PIN_Z + zoff)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=_PIN_TRAVEL, effort=20.0, velocity=0.5),
    )
    model.articulation(
        "ring_swivel",
        ArticulationType.REVOLUTE,
        parent=pin,
        child=ring,
        origin=Origin(xyz=(0.0, _MK2_RING_JOINT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.2, upper=0.9, effort=2.0, velocity=6.0),
    )


def _build_m84_fuze(model: ArticulatedObject, body, r: ResolvedGrenadeConfig, mats: dict) -> None:
    """Twin bent-wire pin + ring fuze above the cyl top cap (S2)."""
    sz = r.body_height_scale
    zoff = _cyl_fuze_zoff(sz)

    body.visual(
        Cylinder(radius=_CYL_FUZE_COLLAR_R, length=_CYL_FUZE_COLLAR_Z1 - _CYL_FUZE_COLLAR_Z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (_CYL_FUZE_COLLAR_Z0 + _CYL_FUZE_COLLAR_Z1) + zoff)),
        material=mats["fuze"],
        name="fuze_collar",
    )
    body.visual(
        Cylinder(radius=_CYL_FUZE_BODY_R, length=_CYL_FUZE_BODY_Z1 - _CYL_FUZE_BODY_Z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (_CYL_FUZE_BODY_Z0 + _CYL_FUZE_BODY_Z1) + zoff)),
        material=mats["fuze"],
        name="fuze_body",
    )
    pivot = (_CYL_LEVER_PIVOT[0], _CYL_LEVER_PIVOT[1], _CYL_LEVER_PIVOT[2] + zoff)
    for i, sy in enumerate((1.0, -1.0)):
        body.visual(
            Box((0.006, 0.0025, 0.006)),
            origin=Origin(xyz=(pivot[0], sy * 0.00775, pivot[2])),
            material=mats["fuze"],
            name=f"lever_lug_{i}",
        )

    lever = model.part("safety_lever")
    lever.visual(
        Cylinder(radius=0.0016, length=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["steel"],
        name="pivot_pin",
    )
    lever.visual(
        Box((0.019, 0.012, 0.002)),
        origin=Origin(xyz=(0.0075, 0.0, 0.0)),
        material=mats["lever"],
        name="top_plate",
    )
    lever.visual(
        Box((0.002, 0.012, 0.081)),
        origin=Origin(xyz=(0.017, 0.0, -0.0395)),
        material=mats["lever"],
        name="side_strap",
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=pivot),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=6.0, lower=0.0, upper=1.57),
    )

    pin1 = model.part("primary_pin")
    _cyl_pin_visuals(pin1, mats["steel"])
    model.articulation(
        "primary_pin_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pin1,
        origin=Origin(xyz=(0.0, _CYL_PIN_MOUNT_R, _CYL_PIN1_Z + zoff)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=_PIN_TRAVEL),
    )
    pin2 = model.part("secondary_pin")
    _cyl_pin_visuals(pin2, mats["steel"])
    d2 = (math.cos(math.radians(165.0)), math.sin(math.radians(165.0)))
    model.articulation(
        "secondary_pin_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pin2,
        origin=Origin(
            xyz=(_CYL_PIN_MOUNT_R * d2[0], _CYL_PIN_MOUNT_R * d2[1], _CYL_PIN2_Z + zoff),
            rpy=(0.0, 0.0, _CYL_PIN2_YAW),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=_PIN_TRAVEL),
    )

    ring1 = model.part("primary_pull_ring")
    ring1_prerot = 0.6
    ring1.visual(
        _cyl_ring_mesh("primary_pull_ring"),
        origin=Origin(rpy=(0.0, ring1_prerot, 0.0)),
        material=mats["steel"],
        name="ring_loop",
    )
    r1_lo, r1_hi = _m84_ring_swing_limits(sz, _CYL_PIN1_Z, ring1_prerot, -1.05, 1.05)
    model.articulation(
        "primary_ring_swing",
        ArticulationType.REVOLUTE,
        parent=pin1,
        child=ring1,
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=r1_lo, upper=r1_hi),
    )
    ring2 = model.part("secondary_pull_ring")
    ring2_prerot = 0.4
    ring2.visual(
        _cyl_ring_mesh("secondary_pull_ring"),
        origin=Origin(rpy=(0.0, ring2_prerot, 0.0)),
        material=mats["steel"],
        name="ring_loop",
    )
    r2_lo, r2_hi = _m84_ring_swing_limits(sz, _CYL_PIN2_Z, ring2_prerot, -1.05, 1.05)
    model.articulation(
        "secondary_ring_swing",
        ArticulationType.REVOLUTE,
        parent=pin2,
        child=ring2,
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=r2_lo, upper=r2_hi),
    )


def _build_bail_fuze(model: ArticulatedObject, body, r: ResolvedGrenadeConfig, mats: dict) -> None:
    """Single pin + triangular bail (child of body) fuze, portable to any spine (S8).

    On the cyl spine the collar/lugs sit above the top cap (S8 geometry).  On a
    lathe spine the same fuze block is re-seated onto the ovoid neck collar
    height so the bail pivot still floats just above the body's fuze collar.
    """
    sz = r.body_height_scale
    if r.is_lathe:
        # Re-seat the m84-style block onto the ovoid neck (body top ~0.085*sz) and
        # stack it up to the mk2 fuze height so the overall body+fuze stays in band.
        # The fuze_body cylinder runs tall enough to anchor the lever + bail lugs
        # (the lugs embed into / seat on it — no floating island).
        collar_z0, collar_z1 = 0.084 * sz, 0.094 * sz
        body_z0, body_z1 = 0.094 * sz, 0.116 * sz
        pivot_z = 0.116 * sz
        lever_pivot_z = 0.112 * sz
        pin_z = 0.106 * sz
    else:
        zoff = _cyl_fuze_zoff(sz)
        collar_z0, collar_z1 = _CYL_FUZE_COLLAR_Z0 + zoff, _CYL_FUZE_COLLAR_Z1 + zoff
        body_z0, body_z1 = _CYL_FUZE_BODY_Z0 + zoff, _CYL_FUZE_BODY_Z1 + zoff
        pivot_z = _CYL_BAIL_PIVOT_Z + zoff
        lever_pivot_z = _CYL_LEVER_PIVOT[2] + zoff
        pin_z = _CYL_PIN1_Z + zoff

    collar_h = collar_z1 - collar_z0
    body_h = body_z1 - body_z0

    body.visual(
        Cylinder(radius=_CYL_FUZE_COLLAR_R, length=collar_h),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (collar_z0 + collar_z1))),
        material=mats["fuze"],
        name="fuze_collar",
    )
    body.visual(
        Cylinder(radius=_CYL_FUZE_BODY_R, length=body_h),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (body_z0 + body_z1))),
        material=mats["fuze"],
        name="fuze_body",
    )
    for i, sy in enumerate((1.0, -1.0)):
        body.visual(
            Box((0.006, 0.0025, 0.006)),
            origin=Origin(xyz=(_CYL_LEVER_PIVOT[0], sy * 0.00775, lever_pivot_z)),
            material=mats["fuze"],
            name=f"lever_lug_{i}",
        )
    for i, sx in enumerate((1.0, -1.0)):
        body.visual(
            Box((0.003, 0.006, 0.005)),
            origin=Origin(xyz=(sx * 0.013, 0.0, pivot_z)),
            material=mats["fuze"],
            name=f"bail_lug_{i}",
        )

    lever = model.part("safety_lever")
    lever.visual(
        Cylinder(radius=0.0016, length=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["steel"],
        name="pivot_pin",
    )
    lever.visual(
        Box((0.019, 0.012, 0.002)),
        origin=Origin(xyz=(0.0075, 0.0, 0.0)),
        material=mats["lever"],
        name="top_plate",
    )
    lever.visual(
        Box((0.002, 0.012, 0.072)),
        origin=Origin(xyz=(0.017, 0.0, -0.0350)),
        material=mats["lever"],
        name="side_strap",
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(_CYL_LEVER_PIVOT[0], 0.0, lever_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=6.0, lower=0.0, upper=1.57),
    )

    pin = model.part("safety_pin")
    _cyl_pin_visuals(pin, mats["steel"])
    model.articulation(
        "safety_pin_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pin,
        origin=Origin(xyz=(0.0, _CYL_PIN_MOUNT_R, pin_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=_PIN_TRAVEL),
    )

    bail = model.part("bail_ring")
    bail.visual(
        _cyl_bail_ring_mesh("bail_wire"), origin=Origin(), material=mats["steel"], name="bail_wire"
    )
    bail.visual(
        Cylinder(radius=_CYL_BAIL_PIVOT_PIN_R, length=_CYL_BAIL_PIVOT_PIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mats["steel"],
        name="pivot_pin",
    )
    model.articulation(
        "bail_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=0.0, upper=2.8),
    )


# ============================================================================
# Top-level builder
# ============================================================================
def build_grenade(
    config: GrenadeConfig, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=config.name)

    pal = _PALETTES[r.palette_style]
    mats = {role: model.material(f"grenade_{role}", rgba=rgba) for role, rgba in pal.items()}

    if r.is_lathe:
        body = _build_lathe_body(model, r, mats)
    else:
        body = _build_cyl_body(model, r, mats)

    if r.fuze == "mk2_single_pin_ring":
        _build_mk2_fuze(model, body, r, mats)
    elif r.fuze == "m84_twin_pin_ring":
        _build_m84_fuze(model, body, r, mats)
    else:
        _build_bail_fuze(model, body, r, mats)

    return model


def build_seeded_grenade(seed: int, *, assets: AssetContext | None = None) -> ArticulatedObject:
    return build_grenade(config_from_seed(seed), assets=assets)


# ============================================================================
# Tests
# ============================================================================
def run_grenade_tests(object_model: ArticulatedObject, config: GrenadeConfig) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)

    root_name = "grenade_body" if r.is_lathe else "body"
    body = object_model.get_part(root_name)

    # ---- intentional captured-pin / interference overlaps (element-scoped) ----
    if r.fuze == "mk2_single_pin_ring":
        lever = object_model.get_part("safety_lever")
        pin = object_model.get_part("safety_pin")
        ring = object_model.get_part("pull_ring")
        ctx.allow_overlap(ring, pin, reason="pull ring threaded through the pin head eye")
        ctx.allow_overlap(
            pin,
            body,
            elem_a=pin.get_visual("cotter_pin"),
            elem_b=body.get_visual("fuze_assembly"),
            reason="pin shaft seated in the housing bore with a light interference fit",
        )
        ctx.allow_overlap(
            lever,
            body,
            elem_a=lever.get_visual("spoon"),
            elem_b=body.get_visual("fuze_assembly"),
            reason="lever pivot ears ride on the fuze lug axle with a light interference fit",
        )
    elif r.fuze == "m84_twin_pin_ring":
        lever = object_model.get_part("safety_lever")
        pin1 = object_model.get_part("primary_pin")
        pin2 = object_model.get_part("secondary_pin")
        ring1 = object_model.get_part("primary_pull_ring")
        ring2 = object_model.get_part("secondary_pull_ring")
        for i in range(2):
            ctx.allow_overlap(
                body,
                lever,
                elem_a=f"lever_lug_{i}",
                elem_b="pivot_pin",
                reason="Hinge pin captured inside the fuze lug ear.",
            )
        for pin in (pin1, pin2):
            ctx.allow_overlap(
                body,
                pin,
                elem_a="fuze_body",
                elem_b="shaft",
                reason="Safety pin shaft seated through the fuze block bore proxy.",
            )
        for pin, ring in ((pin1, ring1), (pin2, ring2)):
            ctx.allow_overlap(
                pin,
                ring,
                elem_a="eye_head",
                elem_b="ring_loop",
                reason="Pull ring wire threaded through the pin eye.",
            )
            ctx.allow_overlap(
                pin,
                ring,
                elem_a="shaft",
                elem_b="ring_loop",
                reason="Pull ring wire passes the shaft end at the pin eye.",
            )
    else:  # single_pin_bail
        lever = object_model.get_part("safety_lever")
        pin = object_model.get_part("safety_pin")
        bail = object_model.get_part("bail_ring")
        for i in range(2):
            ctx.allow_overlap(
                body,
                lever,
                elem_a=f"lever_lug_{i}",
                elem_b="pivot_pin",
                reason="Hinge pin captured inside the fuze lug ear.",
            )
        ctx.allow_overlap(
            body,
            pin,
            elem_a="fuze_body",
            elem_b="shaft",
            reason="Safety pin shaft seated through the fuze block bore proxy.",
        )
        ctx.allow_overlap(
            body,
            bail,
            elem_a="fuze_body",
            elem_b="pivot_pin",
            reason="Bail pivot pin passes through the fuze body swivel mount.",
        )
        for i in range(2):
            ctx.allow_overlap(
                body,
                bail,
                elem_a=f"bail_lug_{i}",
                elem_b="pivot_pin",
                reason="Bail pivot pin captured by the swivel lug ear.",
            )

    # charge tube support rings embed into the ovoid perforated shell wall.
    if r.is_lathe and r.body_surface == "perforated_vent_shell":
        ctx.allow_overlap(
            body,
            body,
            elem_a=body.get_visual("charge_tube"),
            elem_b=body.get_visual("perforated_shell"),
            reason="charge tube support rings seat into the shell inner wall for connectivity",
        )

    # ------------------------- grounding + overall size -------------------------
    bb = ctx.part_world_aabb(body)
    ctx.check("body present aabb", bb is not None, details=f"aabb={bb}")
    if bb is not None:
        (bx0, by0, bz0), (bx1, by1, bz1) = bb
        ctx.check("base grounded at z=0", -1e-3 <= bz0 <= 0.0015, details=f"zmin={bz0}")
        if r.is_lathe:
            # Upper bound covers the tallest realizable lathe stack: the bail
            # fuze at body_height_scale=1.08 reaches ~0.128 (bail lug top), which
            # is a legitimate size, not a defect — band calibrated to the full
            # sampled height-scale range.
            ctx.check("ovoid height in band", 0.105 <= bz1 <= 0.130, details=f"ztop={bz1}")
            ctx.check(
                "ovoid dia x in band", 0.050 <= (bx1 - bx0) <= 0.072, details=f"dx={bx1 - bx0}"
            )
        else:
            ctx.check("cyl height in band", 0.115 <= bz1 <= 0.150, details=f"ztop={bz1}")

    # ------------------------- invariant motion contract -------------------------
    j_lever = object_model.get_articulation("lever_pivot")
    ctx.check(
        "lever is REVOLUTE about -Y, ~90 deg",
        j_lever.articulation_type == ArticulationType.REVOLUTE
        and j_lever.axis[1] < -0.9
        and j_lever.motion_limits is not None
        and abs(j_lever.motion_limits.upper - 1.57) < 0.05
        and abs(j_lever.motion_limits.lower) < 1e-9,
    )

    if r.fuze == "mk2_single_pin_ring":
        j_pin = object_model.get_articulation("pin_slide")
        j_ring = object_model.get_articulation("ring_swivel")
        ctx.check(
            "pin is PRISMATIC +Y 0.02 m",
            j_pin.articulation_type == ArticulationType.PRISMATIC
            and abs(j_pin.motion_limits.upper - _PIN_TRAVEL) < 1e-6
            and j_pin.axis[1] > 0.9,
        )
        ctx.check(
            "ring is REVOLUTE child of pin, span ~2.1",
            j_ring.articulation_type == ArticulationType.REVOLUTE
            and j_ring.parent == "safety_pin"
            and abs((j_ring.motion_limits.upper - j_ring.motion_limits.lower) - 2.1) < 0.05,
        )
    elif r.fuze == "m84_twin_pin_ring":
        for pn in ("primary_pin_slide", "secondary_pin_slide"):
            jp = object_model.get_articulation(pn)
            ctx.check(
                f"{pn} PRISMATIC 0.02 m",
                jp.articulation_type == ArticulationType.PRISMATIC
                and abs(jp.motion_limits.upper - _PIN_TRAVEL) < 1e-6,
            )
        for rn, parent in (
            ("primary_ring_swing", "primary_pin"),
            ("secondary_ring_swing", "secondary_pin"),
        ):
            jr = object_model.get_articulation(rn)
            ctx.check(
                f"{rn} REVOLUTE child of {parent}",
                jr.articulation_type == ArticulationType.REVOLUTE and jr.parent == parent,
            )
            # Derived swing arc must keep the pull-ring loop out of the hex top
            # cap at BOTH extremes (seed-5 穿模 regression guard).
            for edge, val in (("upper", jr.motion_limits.upper), ("lower", jr.motion_limits.lower)):
                clr = min_clearance_at_pose(
                    object_model, joint=rn, joint_positions={rn: val}, keepout=["body"]
                )
                ctx.check(
                    f"{rn} clears the top cap at {edge} limit",
                    clr > 1e-4,
                    details=f"{edge}={val:.4f} clr={clr:.5f}",
                )
    else:
        j_pin = object_model.get_articulation("safety_pin_slide")
        j_bail = object_model.get_articulation("bail_swivel")
        ctx.check(
            "pin is PRISMATIC +Y 0.02 m",
            j_pin.articulation_type == ArticulationType.PRISMATIC
            and abs(j_pin.motion_limits.upper - _PIN_TRAVEL) < 1e-6,
        )
        ctx.check(
            "bail is REVOLUTE -X child of body, 0..2.8",
            j_bail.articulation_type == ArticulationType.REVOLUTE
            and j_bail.parent == root_name
            and j_bail.axis[0] < -0.9
            and abs(j_bail.motion_limits.upper - 2.8) < 0.1,
        )
        sz = r.body_height_scale
        if r.is_lathe:
            collar_top = 0.094 * sz
        else:
            collar_top = _CYL_FUZE_COLLAR_Z1 + _cyl_fuze_zoff(sz)
        ctx.check(
            "bail pivot floats above the fuze collar",
            j_bail.origin.xyz[2] > collar_top,
            details=f"pivot_z={j_bail.origin.xyz[2]}, collar_top={collar_top}",
        )

    # ------------------------- lever release pose -------------------------
    lever_rest = ctx.part_world_aabb(object_model.get_part("safety_lever"))
    with ctx.pose({j_lever: 1.4}):
        lever_open = ctx.part_world_aabb(object_model.get_part("safety_lever"))
    ctx.check(
        "lever swings outward and up on release",
        lever_rest is not None
        and lever_open is not None
        and lever_open[1][0] > lever_rest[1][0] + 0.02
        and lever_open[0][2] > lever_rest[0][2] + 0.02,
        details=f"rest={lever_rest}, open={lever_open}",
    )

    # ------------------------- frag-row multiplicity -------------------------
    if r.body_surface == "frag_knob_grid" and not r.is_lathe:
        # cyl spine exposes per-knob named visuals; verify the count matches N.
        last_row = r.frag_row_count - 1
        ctx.check(
            "frag knob grid emits per-row visuals on cyl spine",
            body.get_visual(f"frag_knob_{last_row}_0") is not None,
            details=f"rows={r.frag_row_count}",
        )

    return ctx.report()
