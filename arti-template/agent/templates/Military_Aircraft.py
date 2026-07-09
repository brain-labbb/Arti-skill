"""Military aircraft — modular procedural template.

Piston-era, propeller-driven, fixed-wing military aircraft (WWII fighter <->
multi-engine warbird family). A grounded lofted fuselage carries a one-piece
fixed wing, a tail empennage (vertical fin + horizontal tail with selectable
movable control surfaces), and N engine/propeller units whose count is the
multiplicity axis.

Frame conventions (shared with every 5-star source):
- +X is the nose (flight) direction, +Y is the port (left) wing, +Z is up.
- The fuselage centerline sits at z = CL_Z so that at rest the lowest geometry
  (resting blade tips, or the wheels with gear-down) grazes the ground plane.
- Articulations: each propeller spins CONTINUOUS about +X; movable empennage
  surfaces hinge REVOLUTE on near-vertical (rudder) or lateral +Y
  (elevator/stabilator) axes.

Slots / module sources (see specs_modular_v1/Military_Aircraft.md):
- Slot A empennage (6 modules): S1 hinged_rudder_plus_elevator, S2 fixed_tall_fin,
  S3 all_moving_stabilator, S4 twin_fin_rudders, S5 split_rudder_off_bomber_fin,
  S6 split_elevator_off_bomber.
- Slot B gear (3 modules): gear_up (S1/S2), fixed_taildragger (S7), fixed_tricycle
  (S8; gated N>=2).
- Slot C engine multiplicity (N in {1,2,3,4}): S9 (N=1 fuselage nose), S2->loop
  (N=2 wing), interpolated N=3, S10 (N=4 wing). Shared loop-rewrite copy
  primitive; props are independent CONTINUOUS +X links (no mimic).

Geometry is adapted verbatim from the 5-star sources (LatheGeometry hulls,
section_loft airfoils) — no Box/Cylinder downgrade. The whole airframe shares
the S1 fighter coordinate frame; gear and engine modules are adapted onto it so
all slot combinations compose in one coherent reference frame.
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
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MatingContract,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    section_loft,
)

__modular__ = True

# ---------------------------------------------------------------------------
# Slot enumerations
# ---------------------------------------------------------------------------
EmpennageModule = Literal[
    "hinged_rudder_plus_elevator",
    "fixed_tall_fin",
    "all_moving_stabilator",
    "twin_fin_rudders",
    "split_rudder_off_bomber_fin",
    "split_elevator_off_bomber",
]
GearModule = Literal["gear_up", "fixed_taildragger", "fixed_tricycle"]
PaletteStyle = Literal[
    "gloss_blue_fighter",
    "bare_metal_bomber",
    "olive_drab_warbird",
    "raf_temperate_camo",
    "navy_sea_blue",
]

EMPENNAGE_MODULES: tuple[EmpennageModule, ...] = (
    "hinged_rudder_plus_elevator",
    "fixed_tall_fin",
    "all_moving_stabilator",
    "twin_fin_rudders",
    "split_rudder_off_bomber_fin",
    "split_elevator_off_bomber",
)
GEAR_MODULES: tuple[GearModule, ...] = ("gear_up", "fixed_taildragger", "fixed_tricycle")
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "gloss_blue_fighter",
    "bare_metal_bomber",
    "olive_drab_warbird",
    "raf_temperate_camo",
    "navy_sea_blue",
)

# Palette colorways. Each maps the same material keys to per-style RGBA. The
# geometry never changes; palette_style is cosmetic only (not a topology slot).
PALETTES: dict[str, dict[str, tuple[float, float, float, float]]] = {
    "gloss_blue_fighter": {
        "body": (0.16, 0.34, 0.62, 1.0),
        "accent": (0.95, 0.78, 0.15, 1.0),
        "marking": (0.93, 0.93, 0.93, 1.0),
        "glass": (0.15, 0.22, 0.32, 1.0),
        "steel": (0.38, 0.39, 0.42, 1.0),
        "engine": (0.13, 0.13, 0.14, 1.0),
        "blade": (0.07, 0.07, 0.08, 1.0),
        "gear_metal": (0.42, 0.43, 0.45, 1.0),
        "rubber": (0.07, 0.07, 0.08, 1.0),
    },
    "bare_metal_bomber": {
        "body": (0.76, 0.77, 0.79, 1.0),
        "accent": (0.94, 0.78, 0.15, 1.0),
        "marking": (0.94, 0.94, 0.94, 1.0),
        "glass": (0.18, 0.24, 0.30, 1.0),
        "steel": (0.16, 0.16, 0.17, 1.0),
        "engine": (0.10, 0.16, 0.38, 1.0),
        "blade": (0.08, 0.08, 0.09, 1.0),
        "gear_metal": (0.42, 0.43, 0.45, 1.0),
        "rubber": (0.07, 0.07, 0.08, 1.0),
    },
    "olive_drab_warbird": {
        "body": (0.30, 0.31, 0.18, 1.0),
        "accent": (0.94, 0.78, 0.15, 1.0),
        "marking": (0.92, 0.92, 0.90, 1.0),
        "glass": (0.16, 0.22, 0.28, 1.0),
        "steel": (0.45, 0.46, 0.48, 1.0),
        "engine": (0.12, 0.12, 0.12, 1.0),
        "blade": (0.07, 0.07, 0.07, 1.0),
        "gear_metal": (0.45, 0.46, 0.48, 1.0),
        "rubber": (0.07, 0.07, 0.08, 1.0),
    },
    "raf_temperate_camo": {
        "body": (0.20, 0.27, 0.16, 1.0),
        "accent": (0.66, 0.12, 0.16, 1.0),
        "marking": (0.92, 0.92, 0.92, 1.0),
        "glass": (0.15, 0.21, 0.27, 1.0),
        "steel": (0.38, 0.39, 0.42, 1.0),
        "engine": (0.11, 0.11, 0.12, 1.0),
        "blade": (0.07, 0.07, 0.08, 1.0),
        "gear_metal": (0.40, 0.41, 0.43, 1.0),
        "rubber": (0.07, 0.07, 0.08, 1.0),
    },
    "navy_sea_blue": {
        "body": (0.12, 0.18, 0.30, 1.0),
        "accent": (0.94, 0.94, 0.94, 1.0),
        "marking": (0.94, 0.94, 0.94, 1.0),
        "glass": (0.13, 0.18, 0.26, 1.0),
        "steel": (0.30, 0.40, 0.52, 1.0),
        "engine": (0.10, 0.10, 0.12, 1.0),
        "blade": (0.07, 0.07, 0.08, 1.0),
        "gear_metal": (0.42, 0.43, 0.45, 1.0),
        "rubber": (0.07, 0.07, 0.08, 1.0),
    },
}

# ---------------------------------------------------------------------------
# Base layout constants (S1 fighter frame, meters). Scales multiply these.
# ---------------------------------------------------------------------------
BASE_CL_Z = 1.08  # fuselage centerline height (gear-up reference)
NOSE_X = 3.40  # cowl front plane
TAIL_X = -4.20  # fuselage tail tip
SPAN_HALF = 5.25  # wing half-span (10.5 m total)
WING_ZC = -0.34  # wing mid-plane below fuselage centerline

PROP_TIP_R = 1.50  # base propeller tip radius (3.0 m disc)
SPINNER_LEN = 0.78
SPINNER_BASE_X = 0.07  # local to the prop joint frame

# Rudder hinge: raked line through (RUDDER_HINGE_X0, 0, 0)
RUDDER_HINGE_X0 = -3.78
RUDDER_HINGE_SLOPE = 0.18
_rud_norm = math.sqrt(1.0 + RUDDER_HINGE_SLOPE**2)
RUDDER_AXIS = (-RUDDER_HINGE_SLOPE / _rud_norm, 0.0, 1.0 / _rud_norm)

ELEV_HINGE_X = -3.80
ELEV_ZC = 0.06

# Stabilator (all-moving tail) pivot (S3)
STAB_PIVOT_X = -3.50
STAB_PIVOT_ZC = 0.06

# Twin-fin endplate placement (S4): fins at the stabilizer tips
TWIN_FIN_Y = 1.75
TWIN_FIN_MOUNT_X = 0.0  # mount at stabilizer station (joint origin handled separately)

# Engine nacelle base geometry (adapted from S9/S10 onto the fighter frame)
NAC_LEN = 1.55
NAC_FRONT_DX = 0.0  # nacelle front face local x (front-face-anchored geometry)
PROP_JOINT_DX = 0.10  # prop joint ahead of the nacelle front face
# Wing nacelle front-face world x. Placed just inside the wing leading edge (the
# wing chord runs forward to ~1.49-1.73 across the nacelle stations) so the
# FIXED joint origin (= the nacelle front face) sits on the wing; the nacelle
# body and prop disc blend into / sweep over the wing (declared overlaps).
WING_NAC_FRONT_X = 1.30
WING_NAC_DZ = WING_ZC + 0.04  # nacelle axis just above wing mid-plane
# Centerline (N=1) nose nacelle front-face world x
NOSE_NAC_FRONT_X = NOSE_X - 0.04

# Landing gear (taildragger, S7) — body/wing frame, relative to centerline
MAIN_GEAR_X = 0.50
MAIN_GEAR_Y = 1.55
MAIN_WHEEL_R = 0.34
MAIN_WHEEL_W = 0.15
TAIL_GEAR_X = -3.50
TAIL_WHEEL_R = 0.12
TAIL_WHEEL_W = 0.05

# Fuselage hull lathe control profile (S1): (radius, z from tail tip)
HULL_PROFILE = [
    (0.015, 0.00),
    (0.07, 0.15),
    (0.13, 0.70),
    (0.20, 1.40),
    (0.30, 2.40),
    (0.42, 3.60),
    (0.52, 4.80),
    (0.60, 5.90),
    (0.65, 6.60),
    (0.66, 7.00),
    (0.655, 7.35),
    (0.60, 7.60),
]


def _hull_radius_at_x(x: float) -> float:
    """Linear-interp hull radius at fuselage station x (centerline coords)."""
    z = x - TAIL_X
    pts = HULL_PROFILE
    if z <= pts[0][1]:
        return pts[0][0]
    for (r0, z0), (r1, z1) in zip(pts, pts[1:]):
        if z0 <= z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + t * (r1 - r0)
    return pts[-1][0]


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class MilitaryAircraftConfig:
    empennage_module: EmpennageModule | None = None
    gear_module: GearModule | None = None
    engine_count: int = 1
    palette_style: PaletteStyle = "gloss_blue_fighter"
    fuselage_len_scale: float = 1.0
    wing_span_scale: float = 1.0
    nac_y_scale: float = 1.0
    prop_radius_scale: float = 1.0
    rudder_throw_deg: float = 30.0
    elevator_throw_deg: float = 25.0
    palette: dict[str, tuple[float, float, float, float]] = field(
        default_factory=lambda: dict(PALETTES["gloss_blue_fighter"])
    )


@dataclass(frozen=True)
class ResolvedMilitaryAircraftConfig:
    empennage_module: EmpennageModule
    gear_module: GearModule
    engine_count: int
    palette_style: PaletteStyle
    fuselage_len_scale: float
    wing_span_scale: float
    nac_y_scale: float
    prop_radius_scale: float
    rudder_throw_rad: float
    elevator_throw_rad: float
    cl_z: float
    span_half: float
    prop_tip_r: float
    nac_y: float  # outboard nacelle |y| for N>=2 (inner pair for N=4)
    nac_stations: tuple[tuple[float, str], ...]  # (y_offset, parent) per engine
    palette: dict[str, tuple[float, float, float, float]]


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(v)))


def _loft(sections, **kw) -> MeshGeometry:
    """section_loft wrapper. Uses repair='off' (skip the expensive CAD kernel
    heal + mesh-repair passes) — the loft kernel already yields a watertight
    airfoil surface here, and healing dominated build time (~20s -> ~0.3s per
    loft). Lets every N-engine seed compile well under the sweep timeout."""
    return section_loft(sections, repair="off", **kw)


# ---------------------------------------------------------------------------
# Mesh helper (collision-safe naming via assets when present)
# ---------------------------------------------------------------------------
def _mesh(model: ArticulatedObject, geometry: MeshGeometry, name: str):
    if model.assets is not None:
        return mesh_from_geometry(geometry, model.assets.mesh_path(f"{name}.obj"))
    return mesh_from_geometry(geometry, name)


# ---------------------------------------------------------------------------
# Airfoil section helpers (S1)
# ---------------------------------------------------------------------------
def _foil_pairs_full(front: float, back: float, ht: float) -> list[tuple[float, float]]:
    """Cambered airfoil loop (rounded LE, near-sharp TE). 17 points."""
    c = front - back
    upper = [
        (0.00, 0.00),
        (0.03, 0.40),
        (0.08, 0.65),
        (0.18, 0.88),
        (0.32, 1.00),
        (0.50, 0.95),
        (0.70, 0.72),
        (0.88, 0.40),
        (1.00, 0.06),
    ]
    lower = [
        (1.00, -0.05),
        (0.88, -0.34),
        (0.70, -0.61),
        (0.50, -0.81),
        (0.32, -0.85),
        (0.18, -0.75),
        (0.08, -0.55),
        (0.03, -0.34),
    ]
    return [(front - u * c, f * ht) for u, f in upper + lower]


def _foil_pairs_blunt_back(front: float, back: float, ht: float) -> list[tuple[float, float]]:
    """Symmetric fixed-surface loop ending blunt at the hinge plane. 13 points."""
    c = front - back
    upper = [
        (0.00, 0.00),
        (0.05, 0.45),
        (0.15, 0.75),
        (0.35, 1.00),
        (0.60, 0.92),
        (0.80, 0.75),
        (1.00, 0.55),
    ]
    lower = [
        (1.00, -0.55),
        (0.80, -0.75),
        (0.60, -0.92),
        (0.35, -1.00),
        (0.15, -0.75),
        (0.05, -0.45),
    ]
    return [(front - u * c, f * ht) for u, f in upper + lower]


def _foil_pairs_blunt_front(front: float, back: float, ht: float) -> list[tuple[float, float]]:
    """Movable control-surface loop: blunt at the hinge, sharp TE. 10 points."""
    c = front - back
    upper = [
        (0.00, 0.55),
        (0.20, 0.50),
        (0.45, 0.38),
        (0.70, 0.24),
        (1.00, 0.05),
    ]
    lower = [
        (1.00, -0.05),
        (0.70, -0.24),
        (0.45, -0.38),
        (0.20, -0.50),
        (0.00, -0.55),
    ]
    return [(front - u * c, f * ht) for u, f in upper + lower]


# ---------------------------------------------------------------------------
# Fuselage / canopy / fairing meshes (S1)
# ---------------------------------------------------------------------------
def _hull_mesh(len_scale: float) -> MeshGeometry:
    profile = sample_catmull_rom_spline_2d(HULL_PROFILE, samples_per_segment=4)
    profile = [(max(r, 0.0), z * len_scale) for r, z in profile]
    profile.append((0.0, profile[-1][1]))  # close the cowl front face
    hull = LatheGeometry(profile, segments=32)
    hull.rotate_y(math.pi / 2.0)  # lathe +Z axis -> +X (nose forward)
    hull.translate(TAIL_X * len_scale, 0.0, 0.0)
    return hull


def _canopy_mesh(len_scale: float) -> MeshGeometry:
    canopy = SphereGeometry(1.0, width_segments=16, height_segments=10)
    canopy.scale(0.78, 0.30, 0.45)
    canopy.translate(0.90 * len_scale, 0.0, 0.45)
    return canopy


def _tail_fairing_mesh(len_scale: float) -> MeshGeometry:
    def loop(z: float, xf: float, xb: float, w: float):
        xf *= len_scale
        xb *= len_scale
        xm = (2.0 * xf + xb) / 3.0
        return [
            (xf + 0.05, 0.0, z),
            (xf, -0.55 * w, z),
            (xm, -w, z),
            (xb + 0.08, -0.7 * w, z),
            (xb, 0.0, z),
            (xb + 0.08, 0.7 * w, z),
            (xm, w, z),
            (xf, 0.55 * w, z),
        ]

    return _loft(
        [
            loop(0.02, -3.55, -4.34, 0.055),
            loop(0.28, -3.62, -4.30, 0.038),
        ]
    )


def _empennage_boss_mesh() -> MeshGeometry:
    """Compact tail-cone hardpoint for fin/stabilizer pivots.

    Keep this visually small: the previous rectangular block satisfied joint
    anchoring but dominated the tail view and visibly cut through the surfaces.
    """

    def loop(x: float, zc: float, half_w: float, half_h: float):
        return [
            (x, 0.0, zc + half_h),
            (x, -0.65 * half_w, zc + 0.55 * half_h),
            (x, -half_w, zc),
            (x, -0.55 * half_w, zc - half_h),
            (x, 0.0, zc - 0.82 * half_h),
            (x, 0.55 * half_w, zc - half_h),
            (x, half_w, zc),
            (x, 0.65 * half_w, zc + 0.55 * half_h),
        ]

    merged = MeshGeometry()
    fairing = _loft(
        [
            loop(-3.95, 0.085, 0.045, 0.055),
            loop(-3.58, 0.120, 0.085, 0.105),
            loop(-3.15, 0.115, 0.060, 0.080),
        ],
        ruled=True,
    )
    merged.merge(fairing)

    # Stabilator / elevator pivot sleeve. It is intentionally narrow and round
    # so the joint reads as hardware instead of a hidden square block.
    sleeve = CylinderGeometry(0.060, 0.42, radial_segments=14)
    sleeve.rotate_x(math.pi / 2.0)
    sleeve.translate(STAB_PIVOT_X, 0.0, STAB_PIVOT_ZC)
    merged.merge(sleeve)
    return merged


def _wing_mesh(span_half: float) -> MeshGeometry:
    # (y_frac, center_x, chord, half_thickness); y scaled to span_half.
    specs = [
        (-1.000, 0.62, 0.30, 0.018),
        (-0.968, 0.66, 0.78, 0.034),
        (-0.867, 0.70, 1.18, 0.052),
        (-0.648, 0.74, 1.50, 0.072),
        (-0.248, 0.78, 1.92, 0.100),
        (0.000, 0.78, 1.98, 0.105),
        (0.248, 0.78, 1.92, 0.100),
        (0.648, 0.74, 1.50, 0.072),
        (0.867, 0.70, 1.18, 0.052),
        (0.968, 0.66, 0.78, 0.034),
        (1.000, 0.62, 0.30, 0.018),
    ]
    sections = []
    for yf, xc, c, ht in specs:
        y = yf * span_half
        pairs = _foil_pairs_full(xc + c / 2.0, xc - c / 2.0, ht)
        sections.append([(x, y, WING_ZC + t) for x, t in pairs])
    return _loft(sections, ruled=True)


# ---------------------------------------------------------------------------
# Single fin + rudder meshes (S1)
# ---------------------------------------------------------------------------
def _fin_le(z: float) -> float:
    return -2.85 - 0.62 * z


def _fin_fixed_back(z: float) -> float:
    return RUDDER_HINGE_X0 - RUDDER_HINGE_SLOPE * z + 0.012


def _fin_ht(z: float) -> float:
    return max(0.018, 0.075 - 0.040 * (z - 0.12))


def _fin_section(z: float) -> list[tuple[float, float, float]]:
    pairs = _foil_pairs_blunt_back(_fin_le(z), _fin_fixed_back(z), _fin_ht(z))
    return [(x, t, z) for x, t in pairs]


def _fin_mesh_body() -> MeshGeometry:
    return _loft([_fin_section(z) for z in (0.12, 0.55, 0.95, 1.30)])


def _fin_mesh_tip() -> MeshGeometry:
    return _loft([_fin_section(z) for z in (1.30, 1.46, 1.55)])


def _rudder_front(z: float) -> float:
    return RUDDER_HINGE_X0 - RUDDER_HINGE_SLOPE * z - 0.012


def _rudder_te(z: float) -> float:
    return -4.33 + 0.05 * z


def _rudder_ht(z: float) -> float:
    return max(0.016, 0.050 - 0.028 * (z - 0.30))


def _rudder_section(z: float) -> list[tuple[float, float, float]]:
    pairs = _foil_pairs_blunt_front(_rudder_front(z), _rudder_te(z), _rudder_ht(z))
    return [(x - RUDDER_HINGE_X0, t, z) for x, t in pairs]


RUDDER_HALF_H = 0.80  # mid-rudder height; the rudder part-local origin sits
# here on the hinge axis (geometry recentred so the joint anchor is on it).


def _rudder_mesh_body() -> MeshGeometry:
    return _loft([_rudder_section(z) for z in (0.30, 0.75, 1.10, 1.30)])


def _rudder_mesh_tip() -> MeshGeometry:
    return _loft([_rudder_section(z) for z in (1.30, 1.44, 1.52)])


def _rudder_hinge_barrel() -> MeshGeometry:
    barrel = CylinderGeometry(0.014, 1.20, radial_segments=10)
    tilt = math.atan2(RUDDER_HINGE_SLOPE, 1.0)
    barrel.rotate_y(-tilt)
    t_mid = RUDDER_HALF_H
    barrel.translate(-RUDDER_HINGE_SLOPE / _rud_norm * t_mid, 0.0, t_mid / _rud_norm)
    return barrel


# ---------------------------------------------------------------------------
# Horizontal stabilizer + elevator (S1) and stabilator (S3)
# ---------------------------------------------------------------------------
def _stab_le(y: float) -> float:
    return -3.05 - 0.28 * abs(y)


def _stab_ht(y: float) -> float:
    return max(0.018, 0.045 - 0.0135 * abs(y))


def _stab_half_mesh(y_sign: float) -> MeshGeometry:
    back = ELEV_HINGE_X + 0.012
    sections = []
    for y_abs in (0.38, 0.72, 1.18, 1.55, 1.78, 1.85):
        y = y_sign * y_abs
        ht = _stab_ht(y) * (0.55 if abs(y) > 1.80 else 1.0)
        pairs = _foil_pairs_blunt_back(_stab_le(y), back, ht)
        sections.append([(x, y, ELEV_ZC + t) for x, t in pairs])
    return _loft(sections)


def _stab_mesh() -> MeshGeometry:
    merged = MeshGeometry()
    merged.merge(_stab_half_mesh(-1.0))
    merged.merge(_stab_half_mesh(1.0))
    return merged


def _elevator_mesh() -> MeshGeometry:
    front = ELEV_HINGE_X - 0.012

    def te(y: float) -> float:
        return -4.28 + 0.22 * abs(y)

    merged = MeshGeometry()
    for y_sign in (-1.0, 1.0):
        sections = []
        for y_abs in (0.42, 0.76, 1.06, 1.32, 1.50):
            y = y_sign * y_abs
            ht = 0.040 - 0.0126 * abs(y)
            pairs = _foil_pairs_blunt_front(front, te(y), ht)
            sections.append([(x - ELEV_HINGE_X, y, t) for x, t in pairs])
        merged.merge(_loft(sections))
    return merged


def _elevator_torque_tube() -> MeshGeometry:
    # Span the full elevator so the tube physically joins both elevator half-
    # panels (inner edges at y=+-0.42) into one connected part; a shorter tube
    # left the panels as disconnected islands.
    tube = CylinderGeometry(0.034, 0.98, radial_segments=10)
    tube.rotate_x(math.pi / 2.0)  # long axis Z -> Y
    return tube


def _stabilator_mesh() -> MeshGeometry:
    """All-moving horizontal tail (S3), authored relative to the pivot frame."""

    def le(y: float) -> float:
        return -3.05 - 0.28 * abs(y)

    def te(y: float) -> float:
        return -4.28 + 0.22 * abs(y)

    def ht(y: float) -> float:
        return max(0.018, 0.045 - 0.0135 * abs(y))

    merged = MeshGeometry()
    for y_sign in (-1.0, 1.0):
        sections = []
        for y_abs in (0.38, 0.72, 1.18, 1.55, 1.78, 1.85):
            y = y_sign * y_abs
            half_t = ht(y) * (0.55 if abs(y) > 1.80 else 1.0)
            pairs = _foil_pairs_full(le(y), te(y), half_t)
            sections.append([(x - STAB_PIVOT_X, y, t) for x, t in pairs])
        merged.merge(_loft(sections))
    return merged


def _stabilator_pivot_shaft() -> MeshGeometry:
    shaft = CylinderGeometry(0.042, 0.85, radial_segments=10)
    shaft.rotate_x(math.pi / 2.0)  # long axis along Y
    return shaft


# ---------------------------------------------------------------------------
# Twin endplate fin + rudder (S4) — authored relative to the stabilizer-tip
# mount, in the fin-local frame (x along chord, y thickness, z height).
# ---------------------------------------------------------------------------
TWIN_HINGE_X0 = -0.55  # rudder hinge x in the fin-local frame
TWIN_HINGE_SLOPE = 0.12
_twin_hnorm = math.sqrt(1.0 + TWIN_HINGE_SLOPE**2)
TWIN_RUDDER_AXIS = (-TWIN_HINGE_SLOPE / _twin_hnorm, 0.0, 1.0 / _twin_hnorm)


def _twin_fin_section(h: float) -> list[tuple[float, float, float]]:
    front = 0.45 - 0.18 * h
    back = TWIN_HINGE_X0 - TWIN_HINGE_SLOPE * h + 0.010
    ht = max(0.016, 0.060 - 0.030 * h)
    pairs = _foil_pairs_blunt_back(front, back, ht)
    return [(x, t, h) for x, t in pairs]


def _twin_fin_mesh_body() -> MeshGeometry:
    return _loft([_twin_fin_section(h) for h in (0.0, 0.40, 0.80, 1.05)])


def _twin_fin_mesh_tip() -> MeshGeometry:
    return _loft([_twin_fin_section(h) for h in (1.05, 1.18, 1.25)])


def _twin_rudder_section(h: float) -> list[tuple[float, float, float]]:
    front = TWIN_HINGE_X0 - TWIN_HINGE_SLOPE * h - 0.010
    te = -1.00 + 0.04 * h
    ht = max(0.014, 0.040 - 0.022 * h)
    pairs = _foil_pairs_blunt_front(front, te, ht)
    return [(x - TWIN_HINGE_X0, t, h) for x, t in pairs]


def _twin_rudder_mesh_body() -> MeshGeometry:
    # h starts at 0 (the hinge base = the part-local origin / joint anchor).
    return _loft([_twin_rudder_section(h) for h in (0.0, 0.40, 0.80, 1.05)])


def _twin_rudder_mesh_tip() -> MeshGeometry:
    return _loft([_twin_rudder_section(h) for h in (1.05, 1.16, 1.22)])


TWIN_RUDDER_HALF_H = 0.55  # mid-height; twin rudder geometry recentred here.


def _twin_rudder_hinge_barrel() -> MeshGeometry:
    barrel = CylinderGeometry(0.013, 0.95, radial_segments=10)
    tilt = math.atan2(TWIN_HINGE_SLOPE, 1.0)
    barrel.rotate_y(-tilt)
    t_mid = TWIN_RUDDER_HALF_H
    barrel.translate(-TWIN_HINGE_SLOPE / _twin_hnorm * t_mid, 0.0, t_mid / _twin_hnorm)
    return barrel


def _recentre_along_twin_rudder_axis(g: MeshGeometry, h: float) -> MeshGeometry:
    g.translate(-TWIN_RUDDER_AXIS[0] * h, 0.0, -TWIN_RUDDER_AXIS[2] * h)
    return g


# ---------------------------------------------------------------------------
# Tall rigid fin three-band loft (S2 fixed_tall_fin) on the fighter frame.
# Surfaces baked into the loft; no movable rudder.
# ---------------------------------------------------------------------------
def _tall_fin_section(z: float) -> list[tuple[float, float, float]]:
    le = -2.70 - 0.78 * z
    back = -4.30 + 0.06 * z
    ht = max(0.018, 0.085 - 0.040 * z)
    pairs = _foil_pairs_blunt_back(le, back, ht)
    return [(x, t, z) for x, t in pairs]


def _tall_fin_mesh_lower() -> MeshGeometry:
    return _loft([_tall_fin_section(z) for z in (0.12, 0.70, 1.30)])


def _tall_fin_mesh_band() -> MeshGeometry:
    return _loft([_tall_fin_section(z) for z in (1.30, 1.55, 1.75)])


def _tall_fin_mesh_upper() -> MeshGeometry:
    return _loft([_tall_fin_section(z) for z in (1.75, 1.95, 2.05)])


def _fin_root_fairing_mesh() -> MeshGeometry:
    def loop(x: float, zc: float, half_w: float, half_h: float):
        return [
            (x, 0.0, zc + half_h),
            (x, -half_w, zc + 0.35 * half_h),
            (x, -0.75 * half_w, zc - half_h),
            (x, 0.0, zc - 0.65 * half_h),
            (x, 0.75 * half_w, zc - half_h),
            (x, half_w, zc + 0.35 * half_h),
        ]

    return _loft(
        [
            loop(-0.78, 0.030, 0.040, 0.055),
            loop(-0.34, 0.075, 0.085, 0.105),
            loop(0.20, 0.055, 0.055, 0.070),
        ],
        ruled=True,
    )


def _rudder_hinge_boss_mesh() -> MeshGeometry:
    boss = CylinderGeometry(0.026, 0.46, radial_segments=10)
    tilt = math.atan2(RUDDER_HINGE_SLOPE, 1.0)
    boss.rotate_y(-tilt)
    return boss


def _stab_root_fairing_mesh(drop: float) -> MeshGeometry:
    lower = -max(0.10, drop + 0.05)

    def loop(y: float, x_half: float, z_top: float, z_bottom: float):
        return [
            (x_half, y, z_top),
            (0.45 * x_half, y, z_top + 0.025),
            (-x_half, y, z_top),
            (-0.75 * x_half, y, 0.5 * (z_top + z_bottom)),
            (-0.35 * x_half, y, z_bottom),
            (0.35 * x_half, y, z_bottom),
            (0.75 * x_half, y, 0.5 * (z_top + z_bottom)),
        ]

    return _loft(
        [
            # Outer loops reach y=+-0.42, past the stabilizer loft inner edge
            # (y=+-0.38), so the center fairing physically bridges the two split
            # stabilizer half-panels into one connected part (they would other-
            # wise be disconnected islands separated by the fuselage gap).
            loop(-0.42, 0.15, 0.020, lower),
            loop(-0.34, 0.15, 0.020, lower),
            loop(0.0, 0.19, 0.035, lower - 0.015),
            loop(0.34, 0.15, 0.020, lower),
            loop(0.42, 0.15, 0.020, lower),
        ],
        ruled=True,
    )


# ---------------------------------------------------------------------------
# Propeller / spinner / nacelle meshes (S1 blades + S9/S10 nacelle)
# ---------------------------------------------------------------------------
def _spinner_mesh() -> MeshGeometry:
    profile = [
        (0.000, 0.00),  # closed base disk so the shaft connects to the shell
        (0.330, 0.00),
        (0.335, 0.10),
        (0.315, 0.28),
        (0.260, 0.48),
        (0.160, 0.66),
        (0.050, 0.76),
        (0.000, SPINNER_LEN),
    ]
    spinner = LatheGeometry(profile, segments=24)
    spinner.rotate_y(math.pi / 2.0)
    spinner.translate(SPINNER_BASE_X, 0.0, 0.0)
    return spinner


def _prop_shaft_mesh() -> MeshGeometry:
    shaft = CylinderGeometry(0.05, 0.30, radial_segments=12)
    shaft.rotate_y(math.pi / 2.0)
    shaft.translate(-0.05, 0.0, 0.0)  # spans x in [-0.20, 0.10]
    return shaft


def _blade_loft(specs: list[tuple[float, float, float, float]]) -> MeshGeometry:
    sections = []
    for r, c, ht, tw in specs:
        pairs = _foil_pairs_full(c / 2.0, -c / 2.0, ht)
        loop = []
        for a, t in pairs:
            x = a * math.sin(tw) + t * math.cos(tw)
            y = a * math.cos(tw) - t * math.sin(tw)
            loop.append((x, y, r))
        sections.append(loop)
    return _loft(sections)


def _blade_set(tip_r: float, blade_count: int) -> MeshGeometry:
    # Blade chord/twist schedule scaled to the requested tip radius.
    base_specs = [
        (0.16, 0.130, 0.075, 1.00),
        (0.30, 0.180, 0.055, 0.85),
        (0.55, 0.225, 0.042, 0.62),
        (0.85, 0.235, 0.033, 0.46),
        (1.12, 0.200, 0.027, 0.37),
        (1.30, 0.165, 0.023, 0.33),
        (1.42, 0.115, 0.018, 0.30),
        (1.50, 0.035, 0.008, 0.28),
    ]
    rs = tip_r / PROP_TIP_R
    specs = [(r * rs, c, ht, tw) for r, c, ht, tw in base_specs]
    base = _blade_loft(specs)
    base.translate(0.30, 0.0, 0.0)  # blade plane behind the spinner mid-body
    merged = MeshGeometry()
    # At rest, distribute blades evenly; offset by 45 deg so 4-blade reads as X.
    phase0 = 45.0 if blade_count == 4 else 90.0
    for k in range(blade_count):
        blade = base.copy()
        blade.rotate_x(math.radians(phase0 + (360.0 / blade_count) * k))
        merged.merge(blade)
    return merged


def _nacelle_hull_mesh() -> MeshGeometry:
    """Wing/fuselage engine nacelle, front-face anchored at local x=0 (S9/S10)."""
    profile = [
        (0.18, 0.00),
        (0.30, 0.10),
        (0.37, 0.30),
        (0.40, 0.60),
        (0.38, 1.00),
        (0.32, 1.30),
        (0.22, 1.50),
        (0.10, NAC_LEN),
    ]
    body = LatheGeometry(profile, segments=24)
    body.rotate_y(math.pi / 2.0)  # +Z -> +X
    body.translate(-NAC_LEN, 0.0, 0.0)  # front face to local x = 0
    return body


def _cowl_ring_mesh() -> MeshGeometry:
    ring = CylinderGeometry(0.40, 0.18, radial_segments=20)
    ring.rotate_y(math.pi / 2.0)
    ring.translate(-0.09, 0.0, 0.0)
    return ring


def _engine_face_mesh() -> MeshGeometry:
    face = CylinderGeometry(0.30, 0.10, radial_segments=18)
    face.rotate_y(math.pi / 2.0)
    face.translate(-0.04, 0.0, 0.0)
    return face


def _antiglare_panel_mesh() -> MeshGeometry:
    panel = BoxGeometry((0.90, 0.34, 0.04))
    panel.translate(-0.60, 0.0, 0.30)
    return panel


# ---------------------------------------------------------------------------
# Landing gear meshes (taildragger S7, tricycle S8)
# ---------------------------------------------------------------------------
def _main_gear_strut_mesh(y_sign: float, wheel_zc: float) -> MeshGeometry:
    y = y_sign * MAIN_GEAR_Y
    merged = MeshGeometry()
    strut_top = WING_ZC - 0.02
    strut_bottom = wheel_zc + 0.06
    mid_z = (strut_top + strut_bottom) / 2.0
    tube = CylinderGeometry(0.045, strut_top - strut_bottom, radial_segments=12)
    tube.translate(MAIN_GEAR_X, y, mid_z)
    merged.merge(tube)
    # drag brace
    brace = CylinderGeometry(0.025, 0.40, radial_segments=8)
    brace.rotate_y(math.radians(40.0))
    brace.translate(MAIN_GEAR_X - 0.16, y, mid_z + 0.10)
    merged.merge(brace)
    # axle
    axle = CylinderGeometry(0.020, MAIN_WHEEL_W + 0.06, radial_segments=10)
    axle.translate(MAIN_GEAR_X, y, wheel_zc)
    merged.merge(axle)
    return merged


def _main_wheel_mesh(y_sign: float, wheel_zc: float) -> MeshGeometry:
    y = y_sign * MAIN_GEAR_Y
    merged = MeshGeometry()
    tire = CylinderGeometry(MAIN_WHEEL_R, MAIN_WHEEL_W, radial_segments=18)
    tire.translate(MAIN_GEAR_X, y, wheel_zc)
    merged.merge(tire)
    hub = CylinderGeometry(MAIN_WHEEL_R * 0.45, MAIN_WHEEL_W + 0.02, radial_segments=12)
    hub.translate(MAIN_GEAR_X, y, wheel_zc)
    merged.merge(hub)
    return merged


def _tail_gear_strut_mesh(wheel_zc: float) -> MeshGeometry:
    merged = MeshGeometry()
    strut_top = -0.10
    strut_bottom = wheel_zc + 0.03
    mid_z = (strut_top + strut_bottom) / 2.0
    strut = CylinderGeometry(0.028, strut_top - strut_bottom, radial_segments=10)
    strut.translate(TAIL_GEAR_X, 0.0, mid_z)
    merged.merge(strut)
    axle = CylinderGeometry(0.012, TAIL_WHEEL_W + 0.03, radial_segments=8)
    axle.translate(TAIL_GEAR_X, 0.0, wheel_zc)
    merged.merge(axle)
    return merged


def _tail_wheel_mesh(wheel_zc: float) -> MeshGeometry:
    tire = CylinderGeometry(TAIL_WHEEL_R, TAIL_WHEEL_W, radial_segments=12)
    tire.translate(TAIL_GEAR_X, 0.0, wheel_zc)
    return tire


def _tricycle_strut_mesh(length: float, radius: float) -> MeshGeometry:
    """Vertical gear strut from local z=0 (mount) down to z=-length."""
    strut = CylinderGeometry(radius, length, radial_segments=10)
    strut.translate(0.0, 0.0, -length / 2.0)
    return strut


def _tricycle_wheel_mesh(length: float, wheel_r: float, wheel_w: float) -> MeshGeometry:
    merged = MeshGeometry()
    tire = CylinderGeometry(wheel_r, wheel_w, radial_segments=16)
    tire.translate(0.0, 0.0, -length)
    merged.merge(tire)
    hub = CylinderGeometry(wheel_r * 0.45, wheel_w + 0.02, radial_segments=10)
    hub.translate(0.0, 0.0, -length)
    merged.merge(hub)
    return merged


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------
def _weighted_engine_count(rng: random.Random) -> int:
    return rng.choices((1, 2, 3, 4), weights=(0.45, 0.35, 0.08, 0.12), k=1)[0]


def config_from_seed(seed: int) -> MilitaryAircraftConfig:
    rng = random.Random(seed)
    n = _weighted_engine_count(rng)
    # Gear: tricycle gated to N>=2 (mains parent to wing nacelles).
    gear_choices = list(GEAR_MODULES)
    if n == 1:
        gear_choices = [g for g in gear_choices if g != "fixed_tricycle"]
    gear = rng.choice(gear_choices)
    empennage = rng.choice(EMPENNAGE_MODULES)
    # Palette: biased by N but all legal.
    if n == 1:
        palette = rng.choices(PALETTE_STYLES, weights=(0.40, 0.10, 0.18, 0.20, 0.12), k=1)[0]
    else:
        palette = rng.choices(PALETTE_STYLES, weights=(0.12, 0.34, 0.26, 0.16, 0.12), k=1)[0]
    return MilitaryAircraftConfig(
        empennage_module=empennage,
        gear_module=gear,
        engine_count=n,
        palette_style=palette,
        fuselage_len_scale=rng.uniform(0.92, 1.10),
        wing_span_scale=rng.uniform(0.90, 1.12),
        nac_y_scale=rng.uniform(0.92, 1.08),
        prop_radius_scale=rng.uniform(0.92, 1.08),
        rudder_throw_deg=rng.uniform(22.0, 34.0),
        elevator_throw_deg=rng.uniform(18.0, 28.0),
        palette=dict(PALETTES[palette]),
    )


def _nacelle_stations(n: int, nac_y: float) -> tuple[tuple[float, str], ...]:
    """(y_offset, parent) per engine unit. Parent=fuselage only for the N=1
    centerline nose; otherwise the wing."""
    if n == 1:
        return ((0.0, "fuselage"),)
    if n == 2:
        return ((nac_y, "wing"), (-nac_y, "wing"))
    if n == 3:
        return ((0.0, "fuselage"), (nac_y, "wing"), (-nac_y, "wing"))
    # n == 4: inner + outer symmetric pairs
    return (
        (nac_y, "wing"),
        (nac_y * 1.85, "wing"),
        (-nac_y, "wing"),
        (-nac_y * 1.85, "wing"),
    )


def resolve_config(config: MilitaryAircraftConfig) -> ResolvedMilitaryAircraftConfig:
    empennage = config.empennage_module or "hinged_rudder_plus_elevator"
    gear = config.gear_module or "gear_up"
    n = int(config.engine_count)
    palette_style = config.palette_style or "gloss_blue_fighter"
    if empennage not in EMPENNAGE_MODULES:
        raise ValueError(f"Unsupported empennage_module: {empennage!r}")
    if gear not in GEAR_MODULES:
        raise ValueError(f"Unsupported gear_module: {gear!r}")
    if palette_style not in PALETTES:
        raise ValueError(f"Unsupported palette_style: {palette_style!r}")
    n = max(1, min(4, n))
    # Compatibility: tricycle requires wing-mounted nacelles (N>=2).
    if gear == "fixed_tricycle" and n == 1:
        gear = "fixed_taildragger"

    len_scale = _clamp(config.fuselage_len_scale, 0.92, 1.10)
    span_scale = _clamp(config.wing_span_scale, 0.90, 1.12)
    nac_y_scale = _clamp(config.nac_y_scale, 0.92, 1.08) if n >= 2 else 1.0
    prop_scale = _clamp(config.prop_radius_scale, 0.92, 1.08)

    prop_tip_r = PROP_TIP_R * prop_scale
    span_half = SPAN_HALF * span_scale

    # Inner-pair nacelle |y|. For multi-engine, keep prop discs from overlapping
    # and clear the fuselage flank; keep outer station inside the wingtip.
    nac_y = 0.0
    if n >= 2:
        nac_y = 2.55 * nac_y_scale
        # prop discs must not overlap: adjacent spacing >= 2 * tip_r + margin.
        min_spacing = 2.0 * prop_tip_r + 0.30
        if n == 4:
            # adjacent stations are nac_y and 1.85*nac_y => spacing 0.85*nac_y.
            min_inner = min_spacing / 0.85
        else:
            # two engines straddle centerline => spacing 2*nac_y.
            min_inner = min_spacing / 2.0
        nac_y = max(nac_y, min_inner)
        # clear fuselage flank: inner prop disc inboard edge stays outside hull.
        nac_y = max(nac_y, prop_tip_r + 0.55)
        # keep outer nacelle inside the wing: span_half >= outer_y + tip margin.
        outer_y = nac_y * (1.85 if n == 4 else 1.0)
        required_span = outer_y + prop_tip_r * 0.30 + 0.40
        if span_half < required_span:
            span_half = required_span

    # Lowest blade tip reach at rest: the 4-blade fighter sits in an X (no blade
    # straight down -> reach = tip_r*cos45), the 3-blade props sit with one blade
    # up (lowest two at +-120deg -> reach = tip_r*sin60). Wing props (N>=2) sit
    # WING_NAC_DZ below the centerline.
    blade_count = 4 if n == 1 else 3
    down_factor = math.cos(math.radians(45.0)) if blade_count == 4 else math.sin(math.radians(60.0))
    blade_reach = prop_tip_r * down_factor
    nac_dz = 0.0 if n == 1 else WING_NAC_DZ  # prop hub z relative to centerline

    # Centerline height so the lowest geometry grazes the ground at rest. For
    # gear-up that is the lowest blade tip; for gear-down the wheels carry it.
    cl_z = max(BASE_CL_Z, blade_reach - nac_dz + 0.04)
    if gear == "fixed_taildragger":
        # Taildragger stance: the wing main wheels reach the ground while the
        # prop disc keeps clear of it; raise the centerline.
        cl_z = max(cl_z, blade_reach - nac_dz + 0.30)

    return ResolvedMilitaryAircraftConfig(
        empennage_module=empennage,
        gear_module=gear,
        engine_count=n,
        palette_style=palette_style,
        fuselage_len_scale=len_scale,
        wing_span_scale=span_scale,
        nac_y_scale=nac_y_scale,
        prop_radius_scale=prop_scale,
        rudder_throw_rad=math.radians(_clamp(config.rudder_throw_deg, 22.0, 34.0)),
        elevator_throw_rad=math.radians(_clamp(config.elevator_throw_deg, 18.0, 28.0)),
        cl_z=cl_z,
        span_half=span_half,
        prop_tip_r=prop_tip_r,
        nac_y=nac_y,
        nac_stations=_nacelle_stations(n, nac_y),
        palette=dict(PALETTES[palette_style]),
    )


# ---------------------------------------------------------------------------
# Build helpers
# ---------------------------------------------------------------------------
def _build_fuselage(model: ArticulatedObject, r: ResolvedMilitaryAircraftConfig):
    cl = Origin(xyz=(0.0, 0.0, r.cl_z))
    fuselage = model.part("fuselage")
    fuselage.visual(
        _mesh(model, _hull_mesh(r.fuselage_len_scale), "fuselage_hull"),
        origin=cl,
        material="body",
        name="hull",
    )
    fuselage.visual(
        Cylinder(radius=0.52, length=0.06),
        origin=Origin(
            xyz=(NOSE_X * r.fuselage_len_scale + 0.02, 0.0, r.cl_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="engine",
        name="engine_face",
    )
    fuselage.visual(
        _mesh(model, _canopy_mesh(r.fuselage_len_scale), "canopy_bubble"),
        origin=cl,
        material="glass",
        name="canopy",
    )
    fuselage.visual(
        _mesh(model, _tail_fairing_mesh(r.fuselage_len_scale), "tail_fairing"),
        origin=cl,
        material="body",
        name="tail_fairing",
    )
    # Empennage root hardpoint: a compact fairing and pivot sleeve at the tail
    # cone. This keeps joint origins grounded without leaving a visible box
    # through the tail surfaces.
    fuselage.visual(
        _mesh(model, _empennage_boss_mesh(), "empennage_boss"),
        origin=cl,
        material="body",
        name="empennage_boss",
    )
    return fuselage


def _anchor_shift(r, ax: float, az: float, ay: float = 0.0) -> Origin:
    """Visual-origin shift so the part-local frame origin lands on geometry at
    world anchor (ax, ay, az), while the centerline-authored mesh keeps its world
    position. The matching FIXED joint origin is (ax, ay, az). This puts every
    FIXED joint origin within tol of both parent and child geometry."""
    return Origin(xyz=(-ax, -ay, r.cl_z - az))


def _wing_anchor_y() -> float:
    """Lateral offset that places the wing FIXED-joint anchor on the fuselage
    hull surface at the wing-root level (the hull is a thin lathe shell, so the
    anchor must sit on the surface, not on the buried centerline). The wing root
    pierces the hull at this y; the wing geometry spans it fully."""
    rad = _hull_radius_at_x(WING_ANCHOR_X)
    return math.sqrt(max(0.0, rad * rad - WING_ZC * WING_ZC)) * 0.96


def _child_shift(r, jx: float, jy: float, jz: float) -> Origin:
    """Visual-origin shift for a child whose mesh is authored in
    centerline-relative coordinates (world = (gx, gy, cl_z + gz)) and whose
    part-local frame origin we want pinned to the world joint anchor
    (jx, jy, jz). Keeps the child geometry in its correct world position."""
    return Origin(xyz=(-jx, -jy, r.cl_z - jz))


# Wing FIXED-joint anchor (centerline-relative). The wing part-local frame
# origin sits here, on the wing root inside the belly. Wing-mounted engine
# origins are expressed relative to this anchor.
WING_ANCHOR_X = 0.30
WING_ANCHOR_Z = WING_ZC  # centerline-relative; world z = cl_z + WING_ZC


def _build_wing(model: ArticulatedObject, r: ResolvedMilitaryAircraftConfig, fuselage):
    # Anchor where the wing root pierces the fuselage hull surface (both the
    # hull shell and the wing reach this point).
    ax = WING_ANCHOR_X
    ay = _wing_anchor_y()
    az = r.cl_z + WING_ANCHOR_Z
    shift = _anchor_shift(r, ax, az, ay)
    wing = model.part("wing")
    wing.visual(
        _mesh(model, _wing_mesh(r.span_half), "wing_loft"),
        origin=shift,
        material="body",
        name="wing_loft",
    )
    # Solid wing-root carry-through block straddling the anchor (a real wing-root
    # fairing). It guarantees the FIXED joint origin sits on solid child
    # geometry (the wing loft is a thin shell) and buries into the belly.
    wing.visual(
        Box((0.9, 0.30, 0.36)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="body",
        name="wing_root_fairing",
    )
    model.articulation(
        "fuselage_to_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=wing,
        origin=Origin(xyz=(ax, ay, az)),
    )
    return wing


# Single-fin FIXED-joint anchor (centerline-relative). The fin part-local frame
# origin sits on the fin root spar inside the tail cone. The rudder hinge is
# re-referenced to this frame.
FIN_ANCHOR_X = -3.30
FIN_ANCHOR_Z = 0.20

# The fin/stabilizer FIXED welds seat on the hull top SKIN (the real fin-root
# mounting interface). The hull is a thin lathe shell, so the weld point must
# land on the outer skin (radius fraction 1.0), not radially inside it — an
# interior fraction leaves the origin tens of mm from the shell wall in the
# hollow, tripping the joint-anchor-honesty gate.
TAIL_ANCHOR_SURFACE_FRAC = 1.0


def _fin_anchor_world_z(r) -> float:
    """World z of the single-fin FIXED-joint anchor (on the hull top surface).

    The hull mesh scales station x by ``fuselage_len_scale`` while its radius
    stays fixed, so the hull ring that lands at world x=FIN_ANCHOR_X comes from
    profile station ``FIN_ANCHOR_X / len_scale`` (same len_scale de-warp the
    nose-gear anchors use). Looking the radius up at the raw x left the anchor
    tens of mm off the real (scaled) tail-cone surface for len_scale != 1."""
    return (
        r.cl_z + _hull_radius_at_x(FIN_ANCHOR_X / r.fuselage_len_scale) * TAIL_ANCHOR_SURFACE_FRAC
    )


def _build_single_fin(model, r, fuselage, *, tall: bool):
    """Vertical fin. tall=True -> rigid three-band tall fin (fixed_tall_fin)."""
    ax = FIN_ANCHOR_X
    # Seat the anchor just on the hull top surface at the fin station (the tail
    # cone is a thin shell); the fin-root fairing box buries down into it.
    az = _fin_anchor_world_z(r)
    shift = _anchor_shift(r, ax, az)
    tail_fin = model.part("tail_fin")
    if tall:
        tail_fin.visual(
            _mesh(model, _tall_fin_mesh_lower(), "fin_lower"),
            origin=shift,
            material="body",
            name="fin_lower",
        )
        tail_fin.visual(
            _mesh(model, _tall_fin_mesh_band(), "fin_band"),
            origin=shift,
            material="accent",
            name="fin_band",
        )
        tail_fin.visual(
            _mesh(model, _tall_fin_mesh_upper(), "fin_upper"),
            origin=shift,
            material="body",
            name="fin_upper",
        )
    else:
        tail_fin.visual(
            _mesh(model, _fin_mesh_body(), "fin_loft"),
            origin=shift,
            material="body",
            name="fin_loft",
        )
        tail_fin.visual(
            _mesh(model, _fin_mesh_tip(), "fin_tip"),
            origin=shift,
            material="accent",
            name="fin_tip",
        )
    # Compact fin-root fairing straddling the anchor (the fin loft is a thin
    # shell), buried in the tail cone and running aft toward the rudder hinge.
    # No origin shift: the fairing straddles the FIXED joint origin so it seats
    # on the hull skin (keeps the joint origin within tol of fin geometry on the
    # child side). It buries into the tail cone / adjacent tail structure, which
    # is declared as allowed overlaps in run_tests.
    tail_fin.visual(
        _mesh(model, _fin_root_fairing_mesh(), "fin_root_fairing"),
        material="body",
        name="fin_root_fairing",
    )
    model.articulation(
        "fuselage_to_tail_fin",
        ArticulationType.FIXED,
        parent=fuselage,
        child=tail_fin,
        origin=Origin(xyz=(ax, 0.0, az)),
    )
    return tail_fin


def _recentre_along_rudder_axis(g: MeshGeometry, h: float) -> MeshGeometry:
    """Translate a rudder mesh down the hinge axis by h so the rudder part-local
    origin lands on the hinge axis at mid-rudder height (on real geometry)."""
    g.translate(-RUDDER_AXIS[0] * h, 0.0, -RUDDER_AXIS[2] * h)
    return g


def _build_rudder(model, r, tail_fin):
    # The rudder mesh is authored in the hinge-local frame (x ~ hinge plane,
    # z = height from the hinge base). Recentre it down the hinge axis so the
    # part-local origin sits at mid-rudder on the hinge axis (on real geometry).
    # The mid-rudder hinge world point is (RUDDER_HINGE_X0, 0, cl_z) + RUDDER_AXIS
    # * RUDDER_HALF_H; express it in the fin part-local frame.
    fin_fx, fin_fz = FIN_ANCHOR_X, _fin_anchor_world_z(r)
    jworld_x = RUDDER_HINGE_X0 + RUDDER_AXIS[0] * RUDDER_HALF_H
    jworld_z = r.cl_z + RUDDER_AXIS[2] * RUDDER_HALF_H
    jx = jworld_x - fin_fx
    jz = jworld_z - fin_fz
    # Hinge-line boss on the fin at the rudder hinge so the REVOLUTE joint origin
    # seats on solid fin geometry (the fin loft is a thin shell at the TE).
    tail_fin.visual(
        _mesh(model, _rudder_hinge_boss_mesh(), "rudder_hinge_boss"),
        origin=Origin(xyz=(jx, 0.0, jz)),
        material="body",
        name="rudder_hinge_boss",
    )
    rudder = model.part("rudder")
    rudder.visual(
        _mesh(
            model, _recentre_along_rudder_axis(_rudder_mesh_body(), RUDDER_HALF_H), "rudder_loft"
        ),
        material="body",
        name="rudder_loft",
    )
    rudder.visual(
        _mesh(model, _recentre_along_rudder_axis(_rudder_mesh_tip(), RUDDER_HALF_H), "rudder_tip"),
        material="accent",
        name="rudder_tip",
    )
    rudder.visual(
        _mesh(
            model,
            _recentre_along_rudder_axis(_rudder_hinge_barrel(), RUDDER_HALF_H),
            "rudder_hinge_barrel",
        ),
        material="steel",
        name="hinge_barrel",
    )
    model.articulation(
        "fin_to_rudder",
        ArticulationType.REVOLUTE,
        parent=tail_fin,
        child=rudder,
        origin=Origin(xyz=(jx, 0.0, jz)),
        axis=RUDDER_AXIS,
        motion_limits=MotionLimits(
            effort=60.0, velocity=3.0, lower=-r.rudder_throw_rad, upper=r.rudder_throw_rad
        ),
        mating=MatingContract(
            # The hinge boss (on the fin) and the rudder hinge barrel are both
            # centred on the hinge axis at the joint origin (captured pin).
            parent_face_geometry="rudder_hinge_boss",
            parent_face_side="negative_x",
            child_face_geometry="hinge_barrel",
            child_face_side="negative_x",
            contact_tol=0.10,
        ),
    )
    return rudder


# Horizontal-stabilizer FIXED-joint anchor (centerline-relative). The stab
# part-local frame origin sits on the stab root inside the tail cone.
STAB_ANCHOR_X = -3.40
STAB_ANCHOR_Z = ELEV_ZC


def _stab_anchor_world_z(r) -> float:
    """World z of the stabilizer FIXED-joint anchor — just on the hull top
    surface at the stab station (so the joint origin sits on the thin hull
    shell)."""
    return (
        r.cl_z + _hull_radius_at_x(STAB_ANCHOR_X / r.fuselage_len_scale) * TAIL_ANCHOR_SURFACE_FRAC
    )


def _build_fixed_stabilizer(model, r, fuselage):
    # Anchor on the hull top surface at the stab station (thin tail-cone shell);
    # the stab-root fairing box bridges down to the stabilizer mid-plane.
    ax, az = STAB_ANCHOR_X, _stab_anchor_world_z(r)
    shift = _anchor_shift(r, ax, az)
    stab = model.part("horizontal_stabilizer")
    stab.visual(
        _mesh(model, _stab_mesh(), "stabilizer_loft"),
        origin=shift,
        material="body",
        name="stabilizer_loft",
    )
    # Compact carry-through fairing from the hull-top anchor down toward the
    # split stabilizer roots.
    drop = az - (r.cl_z + ELEV_ZC)  # anchor height above the stab mid-plane
    stab.visual(
        _mesh(model, _stab_root_fairing_mesh(drop), "stab_root_fairing"),
        material="body",
        name="stab_root_fairing",
    )
    model.articulation(
        "fuselage_to_stabilizer",
        ArticulationType.FIXED,
        parent=fuselage,
        child=stab,
        origin=Origin(xyz=(ax, 0.0, az)),
    )
    return stab


def _build_elevator(model, r, stab):
    # The elevator mesh is authored relative to the hinge (x - ELEV_HINGE_X,
    # z = airfoil thickness); the torque tube sits at the part-local origin, so
    # the joint origin sits on real geometry. Express the hinge world point
    # (ELEV_HINGE_X, 0, cl_z + ELEV_ZC) in the stabilizer part-local frame.
    elevator = model.part("elevator")
    elevator.visual(
        _mesh(model, _elevator_mesh(), "elevator_loft"),
        material="body",
        name="elevator_loft",
    )
    elevator.visual(
        _mesh(model, _elevator_torque_tube(), "elevator_torque_tube"),
        material="steel",
        name="torque_tube",
    )
    # Elevator hinge world point (ELEV_HINGE_X, 0, cl_z + ELEV_ZC) in the
    # stabilizer part-local frame (stab frame origin at world z =
    # _stab_anchor_world_z(r)).
    stab_az = _stab_anchor_world_z(r)
    model.articulation(
        "stabilizer_to_elevator",
        ArticulationType.REVOLUTE,
        parent=stab,
        child=elevator,
        origin=Origin(xyz=(ELEV_HINGE_X - STAB_ANCHOR_X, 0.0, (r.cl_z + ELEV_ZC) - stab_az)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=3.0, lower=-r.elevator_throw_rad, upper=r.elevator_throw_rad
        ),
        mating=MatingContract(
            parent_face_geometry="stabilizer_loft",
            parent_face_side="negative_x",
            child_face_geometry="torque_tube",
            child_face_side="positive_x",
            contact_tol=0.08,
        ),
    )
    return elevator


def _build_stabilator(model, r, fuselage):
    # The stabilator mesh is authored relative to its pivot (x - STAB_PIVOT_X,
    # z = airfoil thickness); the pivot shaft sits at the part-local origin, so
    # the joint origin sits on real geometry. Parent is the fuselage (frame at
    # world origin), so the joint origin is the pivot world point directly.
    jx, jz = STAB_PIVOT_X, r.cl_z + STAB_PIVOT_ZC
    stabilator = model.part("stabilator")
    stabilator.visual(
        _mesh(model, _stabilator_mesh(), "stabilator_loft"),
        material="body",
        name="stabilator_loft",
    )
    stabilator.visual(
        _mesh(model, _stabilator_pivot_shaft(), "stabilator_pivot_shaft"),
        material="steel",
        name="pivot_shaft",
    )
    # Solid pivot hub centred on the part origin so the REVOLUTE joint origin
    # sits on solid geometry (the pivot shaft is a thin tube about the axis).
    stabilator.visual(
        Box((0.12, 0.30, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="steel",
        name="pivot_hub",
    )
    model.articulation(
        "fuselage_to_stabilator",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=stabilator,
        origin=Origin(xyz=(jx, 0.0, jz)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0, velocity=3.0, lower=-r.elevator_throw_rad, upper=r.elevator_throw_rad
        ),
        mating=MatingContract(
            # The empennage boss (on the fuselage) houses the stabilator pivot
            # hub; both straddle the pivot axis at the joint origin (captured
            # pin -> generous contact tol).
            parent_face_geometry="empennage_boss",
            parent_face_side="positive_z",
            child_face_geometry="pivot_hub",
            child_face_side="positive_z",
            contact_tol=0.40,
        ),
    )
    return stabilator


def _build_twin_fins(model, r, stab):
    """Two endplate fins at the stabilizer tips, each with a hinged rudder."""
    fins = []
    rudders = []
    # Fin-mount station in the stabilizer part-local frame (stab frame origin is
    # at world (STAB_ANCHOR_X, 0, _stab_anchor_world_z(r))).
    stab_az = _stab_anchor_world_z(r)
    mount_x = -3.55 - STAB_ANCHOR_X
    mount_z = (r.cl_z + ELEV_ZC) - stab_az
    # Twin rudder hinge anchor: mid-height on the hinge axis in the fin-local
    # frame (the recentred rudder geometry passes through it).
    rud_jx = TWIN_HINGE_X0 + TWIN_RUDDER_AXIS[0] * TWIN_RUDDER_HALF_H
    rud_jz = TWIN_RUDDER_AXIS[2] * TWIN_RUDDER_HALF_H
    for i, y_sign in enumerate((-1.0, 1.0)):
        # Endplate-fin root boss on the stabilizer tip so the FIXED joint origin
        # seats on solid stabilizer geometry (the stab loft is a thin shell).
        stab.visual(
            Box((0.5, 0.30, 0.18)),
            origin=Origin(xyz=(mount_x, y_sign * TWIN_FIN_Y, mount_z)),
            material="body",
            name=f"fin_{i}_root_boss",
        )
        fin_i = model.part(f"fin_{i}")
        fin_i.visual(
            _mesh(model, _twin_fin_mesh_body(), f"fin_{i}_loft"),
            material="body",
            name="fin_loft",
        )
        fin_i.visual(
            _mesh(model, _twin_fin_mesh_tip(), f"fin_{i}_tip"),
            material="accent",
            name="fin_tip",
        )
        model.articulation(
            f"stabilizer_to_fin_{i}",
            ArticulationType.FIXED,
            parent=stab,
            child=fin_i,
            origin=Origin(xyz=(mount_x, y_sign * TWIN_FIN_Y, mount_z)),
        )
        rudder_i = model.part(f"rudder_{i}")
        rudder_i.visual(
            _mesh(
                model,
                _recentre_along_twin_rudder_axis(_twin_rudder_mesh_body(), TWIN_RUDDER_HALF_H),
                f"rudder_{i}_loft",
            ),
            material="body",
            name="rudder_loft",
        )
        rudder_i.visual(
            _mesh(
                model,
                _recentre_along_twin_rudder_axis(_twin_rudder_mesh_tip(), TWIN_RUDDER_HALF_H),
                f"rudder_{i}_tip",
            ),
            material="accent",
            name="rudder_tip",
        )
        rudder_i.visual(
            _mesh(
                model,
                _recentre_along_twin_rudder_axis(_twin_rudder_hinge_barrel(), TWIN_RUDDER_HALF_H),
                f"rudder_{i}_hinge",
            ),
            material="steel",
            name="hinge_barrel",
        )
        model.articulation(
            f"fin_{i}_to_rudder_{i}",
            ArticulationType.REVOLUTE,
            parent=fin_i,
            child=rudder_i,
            origin=Origin(xyz=(rud_jx, 0.0, rud_jz)),
            axis=TWIN_RUDDER_AXIS,
            motion_limits=MotionLimits(
                effort=60.0,
                velocity=3.0,
                lower=-math.radians(28.0),
                upper=math.radians(28.0),
            ),
            mating=MatingContract(
                parent_face_geometry="fin_loft",
                parent_face_side="negative_x",
                child_face_geometry="hinge_barrel",
                child_face_side="positive_x",
                contact_tol=0.16,
            ),
        )
        fins.append(fin_i)
        rudders.append(rudder_i)
    return fins, rudders


def _build_empennage(model, r, fuselage):
    """Dispatch the Slot A empennage module. Returns a dict of key parts."""
    m = r.empennage_module
    out: dict[str, object] = {}
    if m == "hinged_rudder_plus_elevator":
        fin = _build_single_fin(model, r, fuselage, tall=False)
        out["rudder"] = _build_rudder(model, r, fin)
        stab = _build_fixed_stabilizer(model, r, fuselage)
        out["elevator"] = _build_elevator(model, r, stab)
    elif m == "split_rudder_off_bomber_fin":
        # Fixed forward fin + hinged rudder; horizontal tail stays one-piece fixed.
        fin = _build_single_fin(model, r, fuselage, tall=False)
        out["rudder"] = _build_rudder(model, r, fin)
        _build_fixed_stabilizer(model, r, fuselage)
    elif m == "fixed_tall_fin":
        # Tall rigid fin + one-piece fixed stabilizer; no movable surfaces.
        _build_single_fin(model, r, fuselage, tall=True)
        _build_fixed_stabilizer(model, r, fuselage)
    elif m == "all_moving_stabilator":
        fin = _build_single_fin(model, r, fuselage, tall=False)
        out["rudder"] = _build_rudder(model, r, fin)
        out["stabilator"] = _build_stabilator(model, r, fuselage)
    elif m == "twin_fin_rudders":
        stab = _build_fixed_stabilizer(model, r, fuselage)
        out["elevator"] = _build_elevator(model, r, stab)
        fins, rudders = _build_twin_fins(model, r, stab)
        out["twin_fins"] = fins
        out["twin_rudders"] = rudders
    elif m == "split_elevator_off_bomber":
        # Tall rigid fin (fixed) + fixed stabilizer + hinged elevator.
        _build_single_fin(model, r, fuselage, tall=True)
        stab = _build_fixed_stabilizer(model, r, fuselage)
        out["elevator"] = _build_elevator(model, r, stab)
    else:  # pragma: no cover - guarded by resolve_config
        raise ValueError(f"Unsupported empennage_module: {m!r}")
    return out


def _build_engine_unit(model, r, i, y_offset, parent_name, parent_part, blade_count):
    """One nacelle (engine_i) FIXED to its host + an independent CONTINUOUS
    propeller_i about +X (shared loop-rewrite copy primitive, S9/S10).

    The FIXED joint origin sits ON the host surface (wing chord / fuselage cowl).
    The nacelle hull (front face authored at local x=0) is shifted forward by
    `nac_dx` so the spinner protrudes ahead of the host while the nacelle body
    blends back into it. The prop spin joint is placed at the shifted front
    face. This keeps every joint origin within tol of real geometry."""
    # The nacelle hull / engine_face / cowl ring are authored with the front
    # face at local x=0. The FIXED joint origin is placed at the nacelle front
    # face so the joint origin sits on real nacelle geometry (engine_face disk).
    # nac_dx=0: no forward shift; the nacelle body blends back into the host.
    nac_dx = 0.0
    if parent_name == "fuselage":
        # Fuselage nose mount: front face on the cowl (hull reaches it).
        mount_x = NOSE_X * r.fuselage_len_scale - 0.04
        origin = Origin(xyz=(mount_x, 0.0, r.cl_z))
    else:
        # Wing mount: front face at the wing leading edge (on the wing). The
        # wing part-local frame origin is at world (WING_ANCHOR_X,
        # _wing_anchor_y(), cl_z + WING_ANCHOR_Z); express the nacelle mount in
        # that frame so it lands at world (WING_NAC_FRONT_X, y_offset,
        # cl_z + WING_NAC_DZ).
        origin = Origin(
            xyz=(
                WING_NAC_FRONT_X - WING_ANCHOR_X,
                y_offset - _wing_anchor_y(),
                WING_NAC_DZ - WING_ANCHOR_Z,
            )
        )
        # Engine-pylon root boss on the wing at the mount, so the FIXED joint
        # origin seats on solid wing geometry (the wing loft is a thin shell) —
        # Rule 2 anchoring.
        parent_part.visual(
            Box((0.8, 0.40, 0.30)),
            origin=Origin(
                xyz=(
                    WING_NAC_FRONT_X - WING_ANCHOR_X - 0.30,
                    y_offset - _wing_anchor_y(),
                    WING_NAC_DZ - WING_ANCHOR_Z,
                )
            ),
            material="body",
            name=f"engine_{i}_pylon_boss",
        )
    nshift = Origin(xyz=(nac_dx, 0.0, 0.0))

    nacelle = model.part(f"engine_{i}")
    nacelle.visual(
        _mesh(model, _nacelle_hull_mesh(), f"engine_{i}_nacelle_hull"),
        origin=nshift,
        material="body",
        name="nacelle_hull",
    )
    nacelle.visual(
        _mesh(model, _cowl_ring_mesh(), f"engine_{i}_cowl_ring"),
        origin=nshift,
        material="steel",
        name="cowl_ring",
    )
    nacelle.visual(
        _mesh(model, _engine_face_mesh(), f"engine_{i}_engine_face"),
        origin=nshift,
        material="engine",
        name="engine_face",
    )
    nacelle.visual(
        _mesh(model, _antiglare_panel_mesh(), f"engine_{i}_antiglare"),
        origin=nshift,
        material="engine",
        name="antiglare_panel",
    )
    model.articulation(
        f"{parent_name}_to_engine_{i}",
        ArticulationType.FIXED,
        parent=parent_part,
        child=nacelle,
        origin=origin,
    )

    # Prop spin joint at the nacelle front face (on the engine_face / cowl ring,
    # so the joint origin sits on real geometry).
    prop_jx = nac_dx
    prop = model.part(f"propeller_{i}")
    # Solid hub disk centred on the spin axis at the joint origin, so the joint
    # origin sits on real geometry (the shaft is a thin tube around the axis).
    prop.visual(
        Cylinder(radius=0.12, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="body",
        name="prop_hub",
    )
    prop.visual(
        _mesh(model, _spinner_mesh(), f"propeller_{i}_spinner"),
        material="body",
        name="spinner",
    )
    prop.visual(
        _mesh(model, _prop_shaft_mesh(), f"propeller_{i}_prop_shaft"),
        material="steel",
        name="prop_shaft",
    )
    prop.visual(
        _mesh(model, _blade_set(r.prop_tip_r, blade_count), f"propeller_{i}_blades"),
        material="blade",
        name="blades",
    )
    model.articulation(
        f"propeller_{i}_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=prop,
        origin=Origin(xyz=(prop_jx, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=80.0),
    )
    return nacelle, prop


def _build_engines(model, r, fuselage, wing):
    """Slot C multiplicity loop. Returns list of (nacelle, prop, parent_name)."""
    blade_count = 4 if r.engine_count == 1 else 3
    units = []
    for i, (y_offset, parent_name) in enumerate(r.nac_stations):
        parent_part = fuselage if parent_name == "fuselage" else wing
        nacelle, prop = _build_engine_unit(
            model, r, i, y_offset, parent_name, parent_part, blade_count
        )
        units.append((nacelle, prop, parent_name))
    return units


def _build_taildragger_gear(model, r, fuselage, wing):
    """Two main wheels as wing visuals + a tail wheel as a fuselage visual.

    Gear does not articulate, so per Rule 1 it is built as parent visuals on the
    rigid wing / fuselage, not as separate FIXED-jointed parts.
    """
    cl = Origin(xyz=(0.0, 0.0, r.cl_z))
    # The wing part-local frame is at the wing anchor; gear meshes are authored
    # in centerline coords, so apply the same wing shift used for the wing loft.
    wing_shift = _anchor_shift(r, WING_ANCHOR_X, r.cl_z + WING_ANCHOR_Z, _wing_anchor_y())
    # Wheel contact must reach the ground: wheel center at -(cl_z - R) in body z.
    main_wheel_zc = -(r.cl_z - MAIN_WHEEL_R)
    tail_wheel_zc = -(r.cl_z - TAIL_WHEEL_R)
    for gear_i in range(2):
        y_sign = 1.0 - 2.0 * gear_i  # +1 port, -1 starboard
        wing.visual(
            _mesh(model, _main_gear_strut_mesh(y_sign, main_wheel_zc), f"main_gear_strut_{gear_i}"),
            origin=wing_shift,
            material="gear_metal",
            name=f"gear_strut_{gear_i}",
        )
        wing.visual(
            _mesh(model, _main_wheel_mesh(y_sign, main_wheel_zc), f"main_gear_wheel_{gear_i}"),
            origin=wing_shift,
            material="rubber",
            name=f"gear_wheel_{gear_i}",
        )
    fuselage.visual(
        _mesh(model, _tail_gear_strut_mesh(tail_wheel_zc), "tail_gear_strut"),
        origin=cl,
        material="gear_metal",
        name="tail_gear_strut",
    )
    fuselage.visual(
        _mesh(model, _tail_wheel_mesh(tail_wheel_zc), "tail_gear_wheel"),
        origin=cl,
        material="rubber",
        name="tail_gear_wheel",
    )


def _build_tricycle_gear(model, r, fuselage, units):
    """Two main gear parts FIXED to inner wing nacelles + a nose_gear FIXED to
    the fuselage (S8). Requires N>=2 (guaranteed by resolve_config gating)."""
    main_wheel_r = 0.30
    main_wheel_w = 0.14
    nose_wheel_r = 0.22
    nose_wheel_w = 0.11
    # Strut length so wheel bottom reaches z=0 from the nacelle axis height.
    nac_axis_z = r.cl_z + WING_NAC_DZ
    # Mount the strut top just inside the nacelle underside (the nacelle hull
    # mid-body radius is ~0.38) so the gear is solidly attached (no float).
    nac_under = 0.30
    main_strut_len = (nac_axis_z - nac_under) - main_wheel_r
    main_strut_len = max(0.20, main_strut_len)

    # Identify the two inner wing nacelles (smallest |y|, wing parent).
    wing_units = [(u, idx) for idx, (u, _p, pn) in enumerate(_engine_iter(units)) if pn == "wing"]
    # Pick the two innermost by station |y|.
    inner = sorted(wing_units, key=lambda t: abs(r.nac_stations[t[1]][0]))[:2]
    for k, (nacelle, idx) in enumerate(inner):
        gear = model.part(f"gear_{k}")
        gear.visual(
            _mesh(model, _tricycle_strut_mesh(main_strut_len, 0.040), f"gear_{k}_strut"),
            material="gear_metal",
            name=f"gear_{k}_strut",
        )
        gear.visual(
            _mesh(
                model,
                _tricycle_wheel_mesh(main_strut_len, main_wheel_r, main_wheel_w),
                f"gear_{k}_wheel",
            ),
            material="rubber",
            name=f"gear_{k}_wheel",
        )
        # Gear-bay boss on the nacelle at the mount so the FIXED joint origin
        # seats on solid nacelle geometry (the nacelle hull is a thin shell).
        # Wing nacelles use nac_dx=0, so the boss sits at the mount directly.
        nacelle.visual(
            Box((0.30, 0.20, 0.24)),
            origin=Origin(xyz=(-0.70, 0.0, -nac_under)),
            material="body",
            name=f"gear_{k}_bay_boss",
        )
        model.articulation(
            f"nacelle_to_gear_{k}",
            ArticulationType.FIXED,
            parent=nacelle,
            child=gear,
            origin=Origin(xyz=(-0.70, 0.0, -nac_under)),
        )

    # Nose gear FIXED to the forward fuselage.
    nose_gear_x = 1.20 * r.fuselage_len_scale
    nose_under = r.cl_z - _hull_radius_at_x(nose_gear_x / r.fuselage_len_scale) * 0.9
    nose_strut_len = max(0.20, nose_under - nose_wheel_r)
    nose_gear = model.part("nose_gear")
    nose_gear.visual(
        _mesh(model, _tricycle_strut_mesh(nose_strut_len, 0.035), "nose_gear_strut"),
        material="gear_metal",
        name="nose_gear_strut",
    )
    nose_gear.visual(
        _mesh(
            model,
            _tricycle_wheel_mesh(nose_strut_len, nose_wheel_r, nose_wheel_w),
            "nose_gear_wheel",
        ),
        material="rubber",
        name="nose_gear_wheel",
    )
    nose_mount_z = r.cl_z - _hull_radius_at_x(nose_gear_x / r.fuselage_len_scale) * 0.9
    # Nose-gear bay boss on the forward fuselage so the FIXED joint origin seats
    # on solid hull geometry (the hull is a thin lathe shell).
    fuselage.visual(
        Box((0.4, 0.30, 0.30)),
        origin=Origin(xyz=(nose_gear_x, 0.0, nose_mount_z + 0.05)),
        material="body",
        name="nose_gear_bay_boss",
    )
    model.articulation(
        "fuselage_to_nose_gear",
        ArticulationType.FIXED,
        parent=fuselage,
        child=nose_gear,
        origin=Origin(xyz=(nose_gear_x, 0.0, nose_mount_z)),
    )


def _engine_iter(units):
    """Yield (nacelle, prop, parent_name) tuples (units already in that form)."""
    return units


# ---------------------------------------------------------------------------
# Top-level build
# ---------------------------------------------------------------------------
def build_military_aircraft(
    config: MilitaryAircraftConfig, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name="military_aircraft", assets=assets)
    for name, rgba in r.palette.items():
        model.material(name, rgba=rgba)

    fuselage = _build_fuselage(model, r)
    wing = _build_wing(model, r, fuselage)
    _build_empennage(model, r, fuselage)
    units = _build_engines(model, r, fuselage, wing)

    if r.gear_module == "fixed_taildragger":
        _build_taildragger_gear(model, r, fuselage, wing)
    elif r.gear_module == "fixed_tricycle":
        _build_tricycle_gear(model, r, fuselage, units)

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_military_aircraft(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_military_aircraft(config_from_seed(seed), assets=assets)


# ---------------------------------------------------------------------------
# Slot choices
# ---------------------------------------------------------------------------
def slot_choices_for_config(r: ResolvedMilitaryAircraftConfig) -> list[tuple[str, str]]:
    return [
        ("empennage", r.empennage_module),
        ("gear", r.gear_module),
        ("engine_count", f"n{r.engine_count}"),
    ]


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_military_aircraft_tests(
    object_model: ArticulatedObject, config: MilitaryAircraftConfig
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    names = {p.name for p in object_model.parts}
    joints = {j.name: j for j in object_model.articulations}

    fuselage = object_model.get_part("fuselage")
    wing = object_model.get_part("wing")

    # --- always-present identity ---
    ctx.check("fuselage_present", "fuselage" in names)
    ctx.check("wing_present", "wing" in names)
    ctx.check(
        "fuselage_to_wing fixed",
        joints["fuselage_to_wing"].articulation_type == ArticulationType.FIXED,
    )

    # --- empennage present (vertical fin + horizontal tail) ---
    has_single_fin = "tail_fin" in names
    has_twin_fins = "fin_0" in names and "fin_1" in names
    ctx.check("vertical_fin_present", has_single_fin or has_twin_fins)
    has_horizontal_tail = "horizontal_stabilizer" in names or "stabilator" in names
    ctx.check("horizontal_tail_present", has_horizontal_tail)

    # --- intentional, scoped overlaps (real construction junctions) ---
    ctx.allow_overlap(
        wing,
        fuselage,
        elem_a="wing_loft",
        elem_b="hull",
        reason="One-piece low wing carries through the fuselage belly.",
    )

    # Structural root fairings / mounting bosses sit at joint origins; they
    # intentionally bury into the host and adjacent skin. Allow each connector
    # element against the host / mated parts.
    def _elems(part):
        return [v.name for v in part.visuals]

    def _allow_boss(child_part, parent_part, boss_names, *, against_child_too=True):
        for bn in boss_names:
            if bn not in _elems(child_part):
                continue
            for pe in _elems(parent_part):
                ctx.allow_overlap(
                    child_part,
                    parent_part,
                    elem_a=bn,
                    elem_b=pe,
                    reason="Structural root fairing / mounting boss buries into the host.",
                )
            if against_child_too:
                for ce in _elems(child_part):
                    if ce == bn:
                        continue
                    ctx.allow_overlap(
                        child_part,
                        child_part,
                        elem_a=bn,
                        elem_b=ce,
                        reason="Mounting boss is fused into its own part skin.",
                    )

    # Fuselage-borne bosses (empennage boss, nose-gear bay) buried in the hull.
    _allow_boss(fuselage, fuselage, ["empennage_boss", "nose_gear_bay_boss"])
    # Wing root fairing buried in the belly + wing skin.
    _allow_boss(wing, fuselage, ["wing_root_fairing"])
    _allow_boss(wing, wing, ["wing_root_fairing"], against_child_too=True)

    # Empennage seating overlaps depend on the chosen module.
    if has_single_fin:
        tail_fin = object_model.get_part("tail_fin")
        _allow_boss(tail_fin, fuselage, ["fin_root_fairing"])
        _allow_boss(tail_fin, tail_fin, ["fin_root_fairing", "rudder_hinge_boss"])
        fin_elem = (
            "fin_loft"
            if "hinged_rudder_plus_elevator" == r.empennage_module
            or r.empennage_module
            in (
                "split_rudder_off_bomber_fin",
                "all_moving_stabilator",
            )
            else "fin_lower"
        )
        ctx.allow_overlap(
            tail_fin,
            fuselage,
            elem_a=fin_elem,
            elem_b="hull",
            reason="Fin root spar is buried in the tail cone.",
        )
        ctx.allow_overlap(
            tail_fin,
            fuselage,
            elem_a=fin_elem,
            elem_b="tail_fairing",
            reason="Fin root seats into the dorsal tail fairing.",
        )
        ctx.allow_overlap(
            tail_fin,
            fuselage,
            elem_a=fin_elem,
            elem_b="empennage_boss",
            reason="Fin root spar buries into the empennage mounting boss.",
        )

    if "horizontal_stabilizer" in names:
        stab = object_model.get_part("horizontal_stabilizer")
        _allow_boss(stab, fuselage, ["stab_root_fairing"])
        _allow_boss(stab, stab, ["stab_root_fairing"], against_child_too=True)
        ctx.allow_overlap(
            stab,
            fuselage,
            elem_a="stabilizer_loft",
            elem_b="hull",
            reason="One-piece stabilizer carries through the tail cone.",
        )
        ctx.allow_overlap(
            stab,
            fuselage,
            elem_a="stabilizer_loft",
            elem_b="tail_fairing",
            reason="Stabilizer root blends into the dorsal tail fairing.",
        )
        # The stab-root fairing and the vertical fin root share the same tail
        # cone; their buried roots interpenetrate at the empennage junction.
        if has_single_fin:
            tail_fin_part = object_model.get_part("tail_fin")
            fin_root_elems = [
                v.name
                for v in tail_fin_part.visuals
                if v.name in ("fin_loft", "fin_lower", "fin_root_fairing")
            ]
            for fe in fin_root_elems:
                ctx.allow_overlap(
                    stab,
                    tail_fin_part,
                    elem_a="stab_root_fairing",
                    elem_b=fe,
                    reason="Stabilizer and fin roots share the buried tail-cone junction.",
                )

    if "stabilator" in names:
        stabilator = object_model.get_part("stabilator")
        # The all-moving stabilator and its captured pivot pin (shaft + hub) run
        # straight through the tail cone, so they interpenetrate every buried
        # fuselage tail element (hull skin, empennage mounting boss, dorsal
        # fairing) at the empennage junction.
        for se in ("stabilator_loft", "pivot_shaft", "pivot_hub"):
            for fe in ("hull", "empennage_boss", "tail_fairing"):
                ctx.allow_overlap(
                    stabilator,
                    fuselage,
                    elem_a=se,
                    elem_b=fe,
                    reason="Stabilator + captured pivot pin pass through the buried tail cone.",
                )
        # The stabilator pivot hardware shares the tail-cone junction with the
        # vertical fin root; the on-axis hub/shaft buries into the fin root spar.
        if has_single_fin:
            tail_fin_part = object_model.get_part("tail_fin")
            fin_root_elems = [
                v.name
                for v in tail_fin_part.visuals
                if v.name in ("fin_loft", "fin_lower", "fin_root_fairing")
            ]
            for se in ("stabilator_loft", "pivot_shaft", "pivot_hub"):
                for fe in fin_root_elems:
                    ctx.allow_overlap(
                        stabilator,
                        tail_fin_part,
                        elem_a=se,
                        elem_b=fe,
                        reason="Stabilator pivot hub shares the buried tail-cone junction with the fin root.",
                    )

    if "elevator" in names:
        elevator = object_model.get_part("elevator")
        stab = object_model.get_part("horizontal_stabilizer")
        ctx.allow_overlap(
            elevator,
            stab,
            elem_a="torque_tube",
            elem_b="stabilizer_loft",
            reason="Elevator torque tube is captured in the stabilizer hinge line.",
        )
        ctx.allow_overlap(
            elevator,
            fuselage,
            elem_a="elevator_loft",
            elem_b="tail_fairing",
            reason="Elevator root section sits inside the faired hinge slot.",
        )
        ctx.allow_overlap(
            elevator,
            fuselage,
            elem_a="torque_tube",
            elem_b="tail_fairing",
            reason="Elevator torque tube passes through the fairing bore.",
        )
        ctx.allow_overlap(
            elevator,
            fuselage,
            elem_a="elevator_loft",
            elem_b="hull",
            reason="Elevator root passes the tail cone slot.",
        )
        ctx.allow_overlap(
            elevator,
            fuselage,
            elem_a="torque_tube",
            elem_b="hull",
            reason="Elevator torque tube passes through the tail cone bore.",
        )
        ctx.allow_overlap(
            elevator,
            fuselage,
            elem_a="torque_tube",
            elem_b="empennage_boss",
            reason="Elevator torque tube passes through the empennage mounting boss.",
        )

    if "rudder" in names:
        rudder = object_model.get_part("rudder")
        tail_fin = object_model.get_part("tail_fin")
        fin_elems = [v.name for v in tail_fin.visuals]
        for re in ("rudder_loft", "rudder_tip", "hinge_barrel"):
            for fe in fin_elems:
                ctx.allow_overlap(
                    rudder,
                    tail_fin,
                    elem_a=re,
                    elem_b=fe,
                    reason="Rudder root / hinge barrel captured against the fin trailing edge.",
                )
        # The rudder root / hinge barrel sit at the tail cone, grazing the aft
        # fuselage.
        for re in ("rudder_loft", "hinge_barrel"):
            for fe in ("hull", "tail_fairing"):
                ctx.allow_overlap(
                    rudder,
                    fuselage,
                    elem_a=re,
                    elem_b=fe,
                    reason="Rudder root / hinge barrel sweeps into the aft fuselage / tail fairing.",
                )

    if has_twin_fins:
        stab = object_model.get_part("horizontal_stabilizer")
        for i in range(2):
            fin_i = object_model.get_part(f"fin_{i}")
            rudder_i = object_model.get_part(f"rudder_{i}")
            ctx.allow_overlap(
                fin_i,
                stab,
                elem_a="fin_loft",
                elem_b="stabilizer_loft",
                reason="Endplate fin root seats on the stabilizer tip.",
            )
            # The endplate-fin mounting boss (on the stab tip) buries into the
            # fin root spar that it carries.
            ctx.allow_overlap(
                fin_i,
                stab,
                elem_a="fin_loft",
                elem_b=f"fin_{i}_root_boss",
                reason="Endplate fin mounting boss buries into the fin root spar.",
            )
            for re in ("rudder_loft", "rudder_tip", "hinge_barrel"):
                for fe in ("fin_loft", "fin_tip"):
                    ctx.allow_overlap(
                        rudder_i,
                        fin_i,
                        elem_a=re,
                        elem_b=fe,
                        reason="Twin rudder root / hinge barrel captured against its fin TE.",
                    )

    # Engine / propeller overlaps and tip-clearance allowances.
    nac_elems = ("nacelle_hull", "cowl_ring", "engine_face", "antiglare_panel")
    prop_elems = ("prop_shaft", "spinner", "prop_hub", "blades")
    for i, (y_off, parent_name) in enumerate(r.nac_stations):
        nacelle = object_model.get_part(f"engine_{i}")
        prop = object_model.get_part(f"propeller_{i}")
        # Propeller spinner/shaft seat into the cowl bore (captured-pin pattern).
        for pe in prop_elems:
            for ne in nac_elems:
                ctx.allow_overlap(
                    prop,
                    nacelle,
                    elem_a=pe,
                    elem_b=ne,
                    reason="Spinner/prop shaft is captured in the engine cowl bore.",
                )
        if parent_name == "wing":
            # Nacelle (and its cowl/antiglare) blends into the wing carry-through
            # and the engine-pylon root boss.
            wing_targets = ["wing_loft", "wing_root_fairing", f"engine_{i}_pylon_boss"]
            wing_elems = {v.name for v in wing.visuals}
            for ne in nac_elems:
                for wt in wing_targets:
                    if wt in wing_elems:
                        ctx.allow_overlap(
                            nacelle,
                            wing,
                            elem_a=ne,
                            elem_b=wt,
                            reason="Nacelle is mounted on / blended into the wing + pylon boss.",
                        )
            # The large tractor prop disc sweeps over the wing leading edge.
            for pe in prop_elems:
                for wt in wing_targets:
                    if wt in wing_elems:
                        ctx.allow_overlap(
                            prop,
                            wing,
                            elem_a=pe,
                            elem_b=wt,
                            reason="Tractor propeller disc sweeps just ahead of / over the wing LE.",
                        )
        else:
            # Nose nacelle blends into the fuselage cowl; nose prop shaft seats
            # in the fuselage gearbox bore. The long nose cowl also reaches back
            # to the wing carry-through root buried in the belly.
            for ne in nac_elems:
                ctx.allow_overlap(
                    nacelle,
                    fuselage,
                    elem_a=ne,
                    elem_b="hull",
                    reason="Nose nacelle blends into the fuselage cowl.",
                )
                for wt in ("wing_loft", "wing_root_fairing"):
                    ctx.allow_overlap(
                        nacelle,
                        wing,
                        elem_a=ne,
                        elem_b=wt,
                        reason="Long nose cowl reaches the wing carry-through root.",
                    )
                ctx.allow_overlap(
                    nacelle,
                    fuselage,
                    elem_a=ne,
                    elem_b="engine_face",
                    reason="Nose nacelle meets the fuselage cowl face.",
                )
            for pe in prop_elems:
                ctx.allow_overlap(
                    prop,
                    fuselage,
                    elem_a=pe,
                    elem_b="engine_face",
                    reason="Nose propeller seats in the fuselage gearbox bore.",
                )
                ctx.allow_overlap(
                    prop,
                    fuselage,
                    elem_a=pe,
                    elem_b="hull",
                    reason="Nose propeller shaft passes through the cowl into the engine.",
                )

    # Gear embeds
    if r.gear_module == "fixed_taildragger":
        for gear_i in range(2):
            ctx.allow_overlap(
                wing,
                wing,
                elem_a=f"gear_strut_{gear_i}",
                elem_b="wing_loft",
                reason="Main gear strut embeds into the wing underside.",
            )
        ctx.allow_overlap(
            fuselage,
            fuselage,
            elem_a="tail_gear_strut",
            elem_b="hull",
            reason="Tail gear strut embeds into the aft fuselage.",
        )
    elif r.gear_module == "fixed_tricycle":
        # gear_k is FIXED to the k-th innermost wing nacelle (same ordering as
        # the builder). Each gear part embeds into / hangs under its nacelle.
        wing_idxs = sorted(
            [idx for idx, (_y, pn) in enumerate(r.nac_stations) if pn == "wing"],
            key=lambda idx: abs(r.nac_stations[idx][0]),
        )[:2]
        for k in range(2):
            gear = object_model.get_part(f"gear_{k}")
            host = (
                object_model.get_part(f"engine_{wing_idxs[k]}") if k < len(wing_idxs) else fuselage
            )
            for ge in (f"gear_{k}_strut", f"gear_{k}_wheel"):
                for he in _elems(host):
                    ctx.allow_overlap(
                        gear,
                        host,
                        elem_a=ge,
                        elem_b=he,
                        reason="Main gear strut/wheel hangs from the nacelle underside.",
                    )
            # The gear-bay boss is fused into its nacelle skin.
            bn = f"gear_{k}_bay_boss"
            if bn in _elems(host):
                for he in _elems(host):
                    if he != bn:
                        ctx.allow_overlap(
                            host,
                            host,
                            elem_a=bn,
                            elem_b=he,
                            reason="Gear-bay boss is fused into the nacelle skin.",
                        )
        nose_gear = object_model.get_part("nose_gear")
        for ne in ("nose_gear_strut", "nose_gear_wheel"):
            for fe in _elems(fuselage):
                ctx.allow_overlap(
                    nose_gear,
                    fuselage,
                    elem_a=ne,
                    elem_b=fe,
                    reason="Nose gear embeds into the forward fuselage / bay boss.",
                )
        # The nose-gear bay boss sits at the wing carry-through root region.
        if "nose_gear_bay_boss" in _elems(fuselage):
            for we in _elems(wing):
                ctx.allow_overlap(
                    fuselage,
                    wing,
                    elem_a="nose_gear_bay_boss",
                    elem_b=we,
                    reason="Nose-gear bay boss buries into the wing carry-through root.",
                )

    # --- propeller joints: continuous about +X, independent, no mimic ---
    for i in range(r.engine_count):
        spin = joints[f"propeller_{i}_spin"]
        ml = spin.motion_limits
        ctx.check(
            f"propeller_{i} continuous about +X (independent, no limits)",
            spin.articulation_type == ArticulationType.CONTINUOUS
            and abs(spin.axis[0] - 1.0) < 1e-6
            and ml is not None
            and ml.lower is None
            and ml.upper is None
            and getattr(spin, "mimic", None) is None,
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )

    # propellers are independent links: spinning one must not move another.
    if r.engine_count >= 2:
        prop0 = object_model.get_part("propeller_0")
        prop1 = object_model.get_part("propeller_1")
        rest1 = ctx.part_world_aabb(prop1)
        with ctx.pose({joints["propeller_0_spin"]: math.pi / 4.0}):
            moved1 = ctx.part_world_aabb(prop1)
            ctx.check(
                "spinning prop 0 leaves prop 1 unaffected (independent)",
                abs(moved1[1][2] - rest1[1][2]) < 1e-6,
                details=f"rest={rest1[1][2]:.4f}, moved={moved1[1][2]:.4f}",
            )

    # --- 45 deg spin proves the prop is off-axis (real rotation) ---
    prop0 = object_model.get_part("propeller_0")
    prop0_aabb = ctx.part_world_aabb(prop0)
    rest_top = prop0_aabb[1][2]
    with ctx.pose({joints["propeller_0_spin"]: math.pi / 4.0}):
        spun = ctx.part_world_aabb(prop0)
        ctx.check(
            "propeller_0 spin swings blade tips off-axis",
            abs(spun[1][2] - rest_top) > 0.05,
            details=f"rest_top={rest_top:.3f}, spun_top={spun[1][2]:.3f}",
        )

    # --- rudder / elevator / stabilator kinematics ---
    if "fin_to_rudder" in joints:
        rh = joints["fin_to_rudder"]
        rl = rh.motion_limits
        ctx.check(
            "rudder revolute near-vertical hinge within throw",
            rh.articulation_type == ArticulationType.REVOLUTE
            and rh.axis[2] > 0.95
            and abs(rh.axis[1]) < 1e-6
            and rl is not None
            and rl.lower is not None
            and rl.upper is not None,
            details=f"axis={rh.axis}, limits=({rl.lower}, {rl.upper})",
        )
        rudder = object_model.get_part("rudder")
        rud_rest = ctx.part_world_aabb(rudder)
        with ctx.pose({rh: rh.motion_limits.upper}):
            rud_open = ctx.part_world_aabb(rudder)
            ctx.check(
                "rudder swings trailing edge laterally",
                (rud_rest[0][1] - rud_open[0][1]) > 0.10
                or (rud_open[1][1] - rud_rest[1][1]) > 0.10,
                details=f"rest_y=({rud_rest[0][1]:.3f},{rud_rest[1][1]:.3f}), open_y=({rud_open[0][1]:.3f},{rud_open[1][1]:.3f})",
            )

    if "stabilizer_to_elevator" in joints:
        eh = joints["stabilizer_to_elevator"]
        el = eh.motion_limits
        ctx.check(
            "elevator revolute about lateral +Y within throw",
            eh.articulation_type == ArticulationType.REVOLUTE
            and abs(eh.axis[1] - 1.0) < 1e-6
            and el is not None,
            details=f"axis={eh.axis}, limits=({el.lower}, {el.upper})",
        )
        elevator = object_model.get_part("elevator")
        elev_rest = ctx.part_world_aabb(elevator)
        with ctx.pose({eh: eh.motion_limits.upper}):
            elev_up = ctx.part_world_aabb(elevator)
            ctx.check(
                "elevator pitches trailing edge vertically",
                abs(elev_up[1][2] - elev_rest[1][2]) > 0.06,
                details=f"rest_zmax={elev_rest[1][2]:.3f}, up_zmax={elev_up[1][2]:.3f}",
            )

    if "fuselage_to_stabilator" in joints:
        sh = joints["fuselage_to_stabilator"]
        ctx.check(
            "stabilator revolute about lateral +Y",
            sh.articulation_type == ArticulationType.REVOLUTE and abs(sh.axis[1] - 1.0) < 1e-6,
            details=f"axis={sh.axis}",
        )
        stabilator = object_model.get_part("stabilator")
        st_rest = ctx.part_world_aabb(stabilator)
        with ctx.pose({sh: sh.motion_limits.upper}):
            st_up = ctx.part_world_aabb(stabilator)
            ctx.check(
                "stabilator pitches trailing edge vertically",
                abs(st_up[1][2] - st_rest[1][2]) > 0.06,
                details=f"rest_zmax={st_rest[1][2]:.3f}, up_zmax={st_up[1][2]:.3f}",
            )

    if has_twin_fins:
        for i in range(2):
            th = joints[f"fin_{i}_to_rudder_{i}"]
            ctx.check(
                f"twin rudder {i} revolute near-vertical",
                th.articulation_type == ArticulationType.REVOLUTE and th.axis[2] > 0.95,
                details=f"axis={th.axis}",
            )

    # --- engine multiplicity / naming / parent policy ---
    expected_engines = {f"engine_{i}" for i in range(r.engine_count)}
    ctx.check(
        "engine_i naming matches count",
        expected_engines.issubset(names),
        details=f"missing={sorted(expected_engines - names)}",
    )
    for i, (y_off, parent_name) in enumerate(r.nac_stations):
        j = joints[f"{parent_name}_to_engine_{i}"]
        ctx.check(
            f"engine_{i} fixed to {parent_name}",
            j.articulation_type == ArticulationType.FIXED,
            details=f"parent={parent_name}",
        )

    # --- grounding: rest pose grazes the ground plane ---
    all_parts = list(object_model.parts)
    zmin = min(ctx.part_world_aabb(p)[0][2] for p in all_parts)
    ctx.check(
        "rest pose grazes ground plane",
        -0.02 <= zmin <= 0.18,
        details=f"zmin={zmin:.3f}",
    )

    # --- real-world scale ---
    wing_aabb = ctx.part_world_aabb(wing)
    span = wing_aabb[1][1] - wing_aabb[0][1]
    ctx.check(
        "wingspan in realistic range",
        8.0 < span < 35.0,
        details=f"span={span:.3f}",
    )

    return ctx.report()
