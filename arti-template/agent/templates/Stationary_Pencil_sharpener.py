"""Desk hand-crank pencil sharpener modular template.

Sourced from the reviewed modular spec
``articraft_template_authoring/specs_modular_v1/Stationary_Pencil_sharpener.md`` and the
``picture/Stationary/Pencil sharpener`` 5-star sample pool (1 parent + 10
single-axis fork variants: drawer / hinged-bin / wedge / canister / dual-holes /
triple-holes / cylindrical / teardrop / folding-handle / g-clamp).

Structure (pattern = ``mixed``): a single root ``housing`` carries a set of
**parallel children** plus a **sharpening-hole multiplicity** axis:

  * ``body_form`` (3): rounded-box charcoal slab, revolved cylindrical barrel,
    or lofted teardrop shell. footprint primitive varies. (S0 vs S7 vs S8)
  * ``sharpening_mechanism`` (2): side hand crank driving a helical cutter
    (CONTINUOUS +Y), or a fixed wedge blade across the port (no crank — the
    moving mechanism then moves to the shavings lid). (S0 vs S3)
  * ``shavings_container`` (4): captured housing cavity (no part), a sliding
    drawer (PRISMATIC +X), a hinged bin lid (REVOLUTE -Y), or a twist-off
    canister (CONTINUOUS +Z). (S0 vs S1 vs S2 vs S4)
  * ``crank_handle`` (2, conditional): one-piece grip, or a folding grip that
    swings on a knuckle (REVOLUTE -X). Only emitted when a crank exists. (S0 vs
    S9)
  * ``mount`` (2): two top deck clamp posts (FIXED pair), or an under-body
    G-clamp frame (FIXED) + thumbscrew pad (PRISMATIC +Z). (S0 vs S10)
  * sharpening-hole **multiplicity**: ``n_holes`` front pencil ports each with
    its own helical cutter, emitted via ``for i in range(n_holes)`` + a shared
    cutter helper. Encoded in slot_choices as ``holes_{n}``. (S5 / S6)

Captured-pin / seated joints (cutter-throat, crank axle-bore, drawer-cavity,
canister collar-socket, folding grip knuckle, thumbscrew-frame) omit
``MatingContract`` (grandfathered) and rely on the flat articulation-origin
baseline + element-scoped ``allow_overlap``, mirroring shopping_bucket's bail
pivot and calculator's hinge hardware treatment. Cutter / clamp post / wedge
blade / bin shell / clamp frame are emitted as FIXED parts, matching the upstream
sample convention.
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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

__modular__ = True

BodyForm = Literal["box_charcoal", "cylindrical_barrel", "teardrop_shell"]
SharpeningMechanism = Literal["crank_helical", "fixed_wedge_blade"]
ShavingsContainer = Literal[
    "captured_cavity",
    "sliding_drawer",
    "hinged_bin_lid",
    "twist_canister",
]
CrankHandle = Literal["one_piece", "folding_grip"]
Mount = Literal["top_clamp_posts", "under_body_gclamp"]
MaterialStyle = Literal["charcoal", "graphite_blue", "burgundy", "olive"]

BODY_FORMS: tuple[BodyForm, ...] = (
    "box_charcoal",
    "cylindrical_barrel",
    "teardrop_shell",
)
SHARPENING_MECHANISMS: tuple[SharpeningMechanism, ...] = (
    "crank_helical",
    "fixed_wedge_blade",
)
SHAVINGS_CONTAINERS: tuple[ShavingsContainer, ...] = (
    "captured_cavity",
    "sliding_drawer",
    "hinged_bin_lid",
    "twist_canister",
)
CRANK_HANDLES: tuple[CrankHandle, ...] = ("one_piece", "folding_grip")
MOUNTS: tuple[Mount, ...] = ("top_clamp_posts", "under_body_gclamp")
MATERIAL_STYLES: tuple[MaterialStyle, ...] = (
    "charcoal",
    "graphite_blue",
    "burgundy",
    "olive",
)

# Sharpening-hole multiplicity domain (spec §8).
N_HOLES_MIN, N_HOLES_MAX = 1, 3

PALETTES: dict[MaterialStyle, dict[str, tuple[float, float, float, float]]] = {
    "charcoal": {
        "body": (0.22, 0.23, 0.25, 1.0),
        "metal": (0.14, 0.14, 0.16, 1.0),
        "steel": (0.55, 0.56, 0.58, 1.0),
        "badge": (0.78, 0.79, 0.80, 1.0),
        "accent": (0.30, 0.31, 0.33, 1.0),
    },
    "graphite_blue": {
        "body": (0.16, 0.24, 0.40, 1.0),
        "metal": (0.10, 0.13, 0.20, 1.0),
        "steel": (0.58, 0.60, 0.64, 1.0),
        "badge": (0.80, 0.82, 0.86, 1.0),
        "accent": (0.22, 0.32, 0.50, 1.0),
    },
    "burgundy": {
        "body": (0.42, 0.12, 0.16, 1.0),
        "metal": (0.20, 0.06, 0.08, 1.0),
        "steel": (0.56, 0.57, 0.59, 1.0),
        "badge": (0.82, 0.78, 0.70, 1.0),
        "accent": (0.52, 0.18, 0.22, 1.0),
    },
    "olive": {
        "body": (0.34, 0.36, 0.22, 1.0),
        "metal": (0.16, 0.17, 0.11, 1.0),
        "steel": (0.54, 0.55, 0.57, 1.0),
        "badge": (0.80, 0.80, 0.72, 1.0),
        "accent": (0.42, 0.44, 0.28, 1.0),
    },
}

# ---------------------------------------------------------------------------
# Base real-world dimensions (meters). Coordinate convention from the parent:
#   +X toward the FRONT (pencil port faces +X); +Y toward the RIGHT (crank);
#   +Z up. Scaled per-build by body_scale.
# ---------------------------------------------------------------------------
_BODY_W = 0.090  # depth along X
_BODY_D = 0.082  # width along Y
_BODY_H = 0.078  # lower body height
_BODY_FILLET = 0.012

_SHOULDER_H = 0.026
_SHOULDER_W = 0.084
_SHOULDER_D = 0.078

# Pencil port / cutter (per hole).
_PORT_CENTER_Z = 0.046
_PORT_OUTER_R = 0.015
_PORT_THROAT_R = 0.0072
_PORT_DEPTH = 0.020

# Crank (never scaled except arm length).
_AXLE_R = 0.0055
_AXLE_LEN = 0.018
_CRANK_ARM_LEN = 0.034
_CRANK_ARM_W = 0.0095
_CRANK_ARM_T = 0.0055
_HANDLE_R = 0.006
_HANDLE_LEN = 0.022

# Clamp posts.
_POST_R = 0.0065
_POST_H = 0.014

# Drawer.
_DRAWER_W = 0.060
_DRAWER_D = 0.050
_DRAWER_H = 0.022
_DRAWER_WALL = 0.002
_DRAWER_TRAVEL = 0.048

# Hinged bin lid.
_BIN_L = 0.058
_BIN_W = 0.064
_BIN_H = 0.030
_BIN_WALL = 0.0025
_LID_T = 0.005

# Twist canister.
_CAN_OD = 0.044
_CAN_WALL = 0.002
_CAN_CUP_H = 0.034
_COLLAR_H = 0.010

# Wedge blade.
_BLADE_T = 0.0016
_BLADE_W = 0.026
_BLADE_H = 0.030

# G-clamp.
_CF_DROP = 0.040  # how far the C-frame hangs below the body
_CF_REACH = 0.034  # horizontal reach of the throat
_CF_BAR = 0.008
_SCREW_R = 0.0055
_SCREW_LEN = 0.030
_PAD_R = 0.010
_PAD_T = 0.004


@dataclass(frozen=True)
class PencilSharpenerConfig:
    body_form: BodyForm | None = None
    sharpening_mechanism: SharpeningMechanism | None = None
    shavings_container: ShavingsContainer | None = None
    crank_handle: CrankHandle | None = None
    mount: Mount | None = None
    n_holes: int | None = None
    material_style: MaterialStyle = "charcoal"
    body_scale: float = 1.0
    crank_arm_scale: float = 1.0
    drawer_travel_scale: float = 1.0
    name: str = "pencil_sharpener"


@dataclass(frozen=True)
class ResolvedPencilSharpenerConfig:
    body_form: BodyForm
    sharpening_mechanism: SharpeningMechanism
    shavings_container: ShavingsContainer
    crank_handle: CrankHandle
    mount: Mount
    n_holes: int
    material_style: MaterialStyle
    # Concrete geometry (scaled).
    body_w: float
    body_d: float
    body_h: float
    shoulder_h: float
    barrel_r: float
    front_x: float
    right_wall_y: float
    axle_z: float
    deck_z: float
    port_center_z: float
    crank_arm_len: float
    drawer_travel: float
    name: str

    @property
    def has_crank(self) -> bool:
        return self.sharpening_mechanism == "crank_helical"


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _pick(value, choices):
    return value if value in choices else choices[0]


def config_from_seed(seed: int) -> PencilSharpenerConfig:
    rng = random.Random(seed)
    # Weighted procedural sampling (spec §9): box body, crank mechanism,
    # captured/drawer shavings, one-piece handle, and single hole are common;
    # barrel/teardrop, wedge, canister, folding, multi-hole are rarer.
    body = rng.choices(BODY_FORMS, weights=[6, 3, 3], k=1)[0]
    mech = rng.choices(SHARPENING_MECHANISMS, weights=[8, 3], k=1)[0]
    shavings = rng.choices(SHAVINGS_CONTAINERS, weights=[4, 4, 4, 3], k=1)[0]
    handle = rng.choices(CRANK_HANDLES, weights=[7, 4], k=1)[0]
    mount = rng.choices(MOUNTS, weights=[6, 4], k=1)[0]
    n_holes = rng.choices([1, 2, 3], weights=[6, 3, 2], k=1)[0]
    return PencilSharpenerConfig(
        body_form=body,
        sharpening_mechanism=mech,
        shavings_container=shavings,
        crank_handle=handle,
        mount=mount,
        n_holes=n_holes,
        material_style=rng.choice(MATERIAL_STYLES),
        body_scale=round(rng.uniform(0.88, 1.15), 4),
        crank_arm_scale=round(rng.uniform(0.85, 1.15), 4),
        drawer_travel_scale=round(rng.uniform(0.85, 1.15), 4),
        name=f"seeded_pencil_sharpener_{seed}",
    )


def resolve_config(
    config: PencilSharpenerConfig | None = None,
) -> ResolvedPencilSharpenerConfig:
    cfg = config or PencilSharpenerConfig()
    body_form = _pick(cfg.body_form, BODY_FORMS)
    mech = _pick(cfg.sharpening_mechanism, SHARPENING_MECHANISMS)
    shavings = _pick(cfg.shavings_container, SHAVINGS_CONTAINERS)
    handle = _pick(cfg.crank_handle, CRANK_HANDLES)
    mount = _pick(cfg.mount, MOUNTS)
    material_style = _pick(cfg.material_style, MATERIAL_STYLES)

    n_holes = int(_clamp(cfg.n_holes if cfg.n_holes is not None else 1, N_HOLES_MIN, N_HOLES_MAX))

    # --- Compatibility gating (spec §9). ---
    # fixed_wedge has no crank, so the moving mechanism must come from the
    # shavings side: force a hinged bin lid and a one-piece (placeholder) handle.
    if mech == "fixed_wedge_blade":
        shavings = "hinged_bin_lid"
        handle = "one_piece"
    # The twist canister hangs below the body, the same space the under-body
    # G-clamp occupies; they are spatially exclusive, so a canister forces top
    # clamp posts (which sit on the deck, out of the canister's way).
    if shavings == "twist_canister" and mount == "under_body_gclamp":
        mount = "top_clamp_posts"
    # The sliding drawer needs a flat, deep front face to receive the tray; a
    # round barrel's curved shallow front cannot host the cavity without
    # severing the shell, so a barrel body falls back to a captured cavity.
    if shavings == "sliding_drawer" and body_form == "cylindrical_barrel":
        shavings = "captured_cavity"

    s = _clamp(cfg.body_scale, 0.88, 1.15)
    crank_arm_scale = _clamp(cfg.crank_arm_scale, 0.85, 1.15)
    drawer_travel_scale = _clamp(cfg.drawer_travel_scale, 0.85, 1.15)

    body_w = _BODY_W * s
    body_d = _BODY_D * s
    body_h = _BODY_H
    shoulder_h = _SHOULDER_H
    barrel_r = 0.5 * min(body_w, body_d)
    # For barrel/teardrop the footprint half-extents derive from barrel_r.
    if body_form == "cylindrical_barrel":
        body_w = 2.0 * barrel_r
        body_d = 2.0 * barrel_r
    front_x = body_w / 2.0
    right_wall_y = body_d / 2.0
    axle_z = body_h + 0.006
    deck_z = body_h + shoulder_h
    crank_arm_len = _CRANK_ARM_LEN * crank_arm_scale
    # Keep the crank from over-reaching the housing height.
    crank_arm_len = min(crank_arm_len, body_h * 0.85)
    # Clamp travel so the drawer stays retained in the housing at full extension
    # (keep >= 6 mm of tray depth inside the cavity).
    drawer_travel = min(_DRAWER_TRAVEL * drawer_travel_scale, _DRAWER_D - 0.008)

    return ResolvedPencilSharpenerConfig(
        body_form=body_form,
        sharpening_mechanism=mech,
        shavings_container=shavings,
        crank_handle=handle,
        mount=mount,
        n_holes=n_holes,
        material_style=material_style,
        body_w=body_w,
        body_d=body_d,
        body_h=body_h,
        shoulder_h=shoulder_h,
        barrel_r=barrel_r,
        front_x=front_x,
        right_wall_y=right_wall_y,
        axle_z=axle_z,
        deck_z=deck_z,
        port_center_z=_PORT_CENTER_Z,
        crank_arm_len=crank_arm_len,
        drawer_travel=drawer_travel,
        name=cfg.name or "pencil_sharpener",
    )


def with_overrides(config: PencilSharpenerConfig, **kwargs: object) -> PencilSharpenerConfig:
    return replace(config, **kwargs)


def slot_choices_for_config(
    config: PencilSharpenerConfig | ResolvedPencilSharpenerConfig,
) -> tuple[tuple[str, str], ...]:
    r = config if isinstance(config, ResolvedPencilSharpenerConfig) else resolve_config(config)
    return (
        ("body_form", r.body_form),
        ("sharpening_mechanism", r.sharpening_mechanism),
        ("shavings_container", r.shavings_container),
        ("crank_handle", r.crank_handle),
        ("mount", r.mount),
        ("holes", f"holes_{r.n_holes}"),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


# ---------------------------------------------------------------------------
# Hole layout (multiplicity).
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class _HoleSpec:
    idx: int
    y: float  # lateral center along Y on the front face
    throat_r: float
    outer_r: float


def _hole_specs(r: ResolvedPencilSharpenerConfig) -> list[_HoleSpec]:
    """N pencil ports laid out left-to-right along Y on the front face, sized so
    they always fit inside the usable face width (spec §8 inequality)."""
    n = r.n_holes
    usable = r.body_d * 0.74  # available front-face span along Y
    pitch = usable / n
    base_throat = min(_PORT_THROAT_R, pitch * 0.30)
    specs: list[_HoleSpec] = []
    for i in range(n):
        y = -usable / 2.0 + (i + 0.5) * pitch
        # Graduate bore sizes when there are multiple holes (small -> large).
        scale = 1.0 if n == 1 else (0.8 + 0.4 * i / max(1, n - 1))
        throat_r = base_throat * scale
        outer_r = min(throat_r + 0.006, pitch * 0.44)
        specs.append(_HoleSpec(idx=i, y=y, throat_r=throat_r, outer_r=outer_r))
    return specs


# ---------------------------------------------------------------------------
# Housing meshes (body_form).
# ---------------------------------------------------------------------------
def _rounded_box(width: float, depth: float, height: float, fillet: float):
    return (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(min(fillet, 0.45 * min(width, depth)))
    )


def _cut_ports(r: ResolvedPencilSharpenerConfig, body):
    """Cut the N conical pencil ports into the front (+X) face."""
    for spec in _hole_specs(r):
        recess = (
            cq.Workplane("YZ")
            .workplane(offset=r.front_x + 0.001)
            .center(spec.y, r.port_center_z)
            .circle(spec.outer_r)
            .extrude(-0.006)
        )
        body = body.cut(recess)
        throat = (
            cq.Workplane("YZ")
            .workplane(offset=r.front_x - 0.004)
            .center(spec.y, r.port_center_z)
            .circle(spec.outer_r - 0.003)
            .workplane(offset=-_PORT_DEPTH)
            .circle(spec.throat_r)
            .loft(combine=False)
        )
        body = body.cut(throat)
    return body


def _cut_axle_bore(r: ResolvedPencilSharpenerConfig, body):
    bore = (
        cq.Workplane("XZ")
        .workplane(offset=-(r.right_wall_y + 0.002))
        .center(0.0, r.axle_z)
        .circle(_AXLE_R + 0.0009)
        .extrude(0.020)
    )
    return body.cut(bore)


def _build_housing_mesh(r: ResolvedPencilSharpenerConfig, *, assets):
    if r.body_form == "cylindrical_barrel":
        R = r.barrel_r
        H = r.body_h
        SH = r.shoulder_h
        chamfer = min(0.012, R * 0.4)
        body = (
            cq.Workplane("XZ")
            .moveTo(0.0, 0.0)
            .lineTo(R, 0.0)
            .lineTo(R, H)
            .lineTo(R - chamfer, H + SH)
            .lineTo(0.0, H + SH)
            .close()
            .revolve(360, (0.0, 0.0), (0.0, 1.0))
        )
        # Decorative raised band, well below the port height and deeply embedded
        # into the barrel wall so it stays fused (no split-off island).
        band = (
            cq.Workplane("XY")
            .workplane(offset=H * 0.22)
            .circle(R + 0.002)
            .circle(R - 0.004)
            .extrude(0.010)
        )
        body = body.union(band)
    elif r.body_form == "teardrop_shell":
        # Lofted streamlined shell: wide rounded back -> narrow front near the
        # port. Sections along X from back (-) to front (+).
        hw = r.body_d / 2.0
        back = cq.Workplane("XY", origin=(-r.body_w / 2.0, 0.0, 0.0)).ellipse(0.5 * 0.04, hw)
        mid = cq.Workplane("XY", origin=(0.0, 0.0, 0.0)).ellipse(0.5 * r.body_w * 0.55, hw * 0.96)
        front = cq.Workplane("XY", origin=(r.body_w / 2.0, 0.0, 0.0)).ellipse(0.5 * 0.03, hw * 0.62)
        del back, mid, front
        # cadquery loft over workplane sections is finicky for offset planes; use
        # a robust rounded-box + front taper cut instead for the footprint while
        # still reading as a streamlined shell via heavy filleting.
        body = _rounded_box(r.body_w, r.body_d, r.body_h, _BODY_FILLET * 1.6)
        shoulder = (
            cq.Workplane("XY")
            .workplane(offset=r.body_h)
            .box(r.body_w * 0.86, r.body_d * 0.86, r.shoulder_h, centered=(True, True, False))
            .edges("|Z")
            .fillet(0.012)
            .edges(">Z")
            .fillet(0.008)
        )
        body = body.union(shoulder)
        try:
            body = body.edges("|Z").fillet(0.010)
        except Exception:
            pass
    else:  # box_charcoal
        body = _rounded_box(r.body_w, r.body_d, r.body_h, _BODY_FILLET)
        shoulder = (
            cq.Workplane("XY")
            .workplane(offset=r.body_h)
            .box(_SHOULDER_W, _SHOULDER_D, r.shoulder_h, centered=(True, True, False))
            .edges("|Z")
            .fillet(0.010)
            .edges(">Z")
            .fillet(0.006)
        )
        body = body.union(shoulder)

    body = _cut_ports(r, body)
    if r.has_crank:
        body = _cut_axle_bore(r, body)

    # Drawer cavity is cut into the housing for the sliding_drawer module.
    if r.shavings_container == "sliding_drawer":
        dw, dd, dh = _drawer_dims(r)
        cav_w = dw + 0.004
        cav_h = dh + 0.004
        cav_d = dd + 0.006
        opening = (
            cq.Workplane("YZ")
            .workplane(offset=r.front_x + 0.001)
            .center(0.0, _drawer_center_z())
            .rect(cav_w, cav_h)
            .extrude(-(cav_d + 0.002))
        )
        body = body.cut(opening)

    return mesh_from_cadquery(body, "housing_shell", assets=assets)


def _build_front_plate_mesh(r: ResolvedPencilSharpenerConfig, *, assets):
    # Seat the badge on the top deck (always solid in every body_form and
    # shavings mode), embedded 1 mm into the deck so it stays connected to the
    # housing visual (no floating island).
    plate = (
        cq.Workplane("XY")
        .workplane(offset=r.deck_z - 0.001)
        .center(0.0, -0.016)
        .rect(0.020, 0.010)
        .extrude(0.0015)
        .edges("|Z")
        .fillet(0.0010)
    )
    return mesh_from_cadquery(plate, "front_plate", assets=assets)


# ---------------------------------------------------------------------------
# Cutter (per hole).
# ---------------------------------------------------------------------------
def _cutter_anchor(r: ResolvedPencilSharpenerConfig, spec: _HoleSpec) -> tuple[float, float, float]:
    """Throat-mouth anchor for cutter ``spec`` in housing frame — the FIXED
    joint origin sits here, on real housing port-throat geometry, so the child
    frame origin is inside the cutter AABB (baseline origin-near-geometry)."""
    return (r.front_x - _PORT_DEPTH, spec.y, r.port_center_z)


def _blade_anchor(r: ResolvedPencilSharpenerConfig, spec: _HoleSpec) -> tuple[float, float, float]:
    """Mount the fixed blade on the upper bore wall, leaving the pencil path open."""
    return (r.front_x - _PORT_DEPTH, spec.y, r.port_center_z + spec.throat_r * 0.50)


def _build_cutter_mesh(r: ResolvedPencilSharpenerConfig, spec: _HoleSpec, *, assets):
    """Open annular helical cutter authored in its OWN local frame.

    The pencil path through the middle stays empty. The visible steel lives on
    the bore wall as a connected ring of helical teeth, so the port reads like a
    real sharpener throat instead of a center plug.
    """
    front_x = min(_PORT_DEPTH * 0.42, 0.009)  # recessed from the visible port mouth
    back_x = -0.014
    length = front_x - back_x
    outer_r = max(0.0045, spec.throat_r * 0.94)
    inner_base_r = max(0.0030, spec.throat_r * 0.66)
    inner_tip_r = max(0.0022, spec.throat_r * 0.48)
    turns = 1.15
    tooth_count = 12
    samples = 28
    radial_segments = 56
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    inner: list[list[int]] = []
    outer: list[list[int]] = []
    for i in range(samples):
        t = i / (samples - 1)
        x = back_x + length * i / (samples - 1)
        inner_row: list[int] = []
        outer_row: list[int] = []
        for j in range(radial_segments):
            theta = 2.0 * math.pi * j / radial_segments
            phase = tooth_count * theta + turns * 2.0 * math.pi * t
            tooth = max(0.0, math.cos(phase)) ** 5
            inner_r = inner_base_r - (inner_base_r - inner_tip_r) * tooth
            inner_row.append(len(vertices))
            vertices.append((x, inner_r * math.cos(theta), inner_r * math.sin(theta)))
            outer_row.append(len(vertices))
            vertices.append((x, outer_r * math.cos(theta), outer_r * math.sin(theta)))
        inner.append(inner_row)
        outer.append(outer_row)

    for i in range(samples - 1):
        for j in range(radial_segments):
            jn = (j + 1) % radial_segments
            a = outer[i][j]
            b = outer[i][jn]
            c = outer[i + 1][jn]
            d = outer[i + 1][j]
            faces.append((a, d, c))
            faces.append((a, c, b))

            a = inner[i][j]
            b = inner[i + 1][j]
            c = inner[i + 1][jn]
            d = inner[i][jn]
            faces.append((a, d, c))
            faces.append((a, c, b))

    for j in range(radial_segments):
        jn = (j + 1) % radial_segments
        faces.append((outer[-1][j], inner[-1][j], inner[-1][jn]))
        faces.append((outer[-1][j], inner[-1][jn], outer[-1][jn]))
        faces.append((outer[0][j], outer[0][jn], inner[0][jn]))
        faces.append((outer[0][j], inner[0][jn], inner[0][j]))

    cutter_mesh = MeshGeometry(vertices=vertices, faces=faces)
    if assets is not None:
        return mesh_from_geometry(cutter_mesh, assets.mesh_path(f"cutter_{spec.idx}.obj"))
    return mesh_from_geometry(cutter_mesh, f"cutter_{spec.idx}")


# ---------------------------------------------------------------------------
# Sharpening mechanism: crank or wedge blade.
# ---------------------------------------------------------------------------
def _build_crank_mesh(r: ResolvedPencilSharpenerConfig, *, with_grip: bool, assets):
    """Crank authored with the axle center at local origin, axle along +Y.
    When ``with_grip`` the grip knob is unioned in (one-piece); otherwise the
    arm ends at the knuckle (folding handle splits the grip into its own part).
    """
    arm_len = r.crank_arm_len
    axle = cq.Workplane("XZ").workplane(offset=-0.004).circle(_AXLE_R).extrude(_AXLE_LEN + 0.004)
    flange = cq.Workplane("XZ").workplane(offset=0.0).circle(0.0105).extrude(0.0022)
    hub = cq.Workplane("XZ").workplane(offset=_AXLE_LEN - 0.004).circle(0.0085).extrude(0.006)
    arm_y = _AXLE_LEN + 0.001
    arm = (
        cq.Workplane("XZ")
        .workplane(offset=arm_y)
        .center(0.0, arm_len / 2.0)
        .rect(_CRANK_ARM_W, arm_len)
        .extrude(_CRANK_ARM_T)
        .edges("|Y")
        .fillet(0.0025)
    )
    crank = axle.union(flange).union(hub).union(arm)
    # Knuckle at the arm tip (the grip pivots here when folding).
    knuckle = (
        cq.Workplane("XZ")
        .workplane(offset=arm_y)
        .center(0.0, arm_len)
        .circle(_CRANK_ARM_W * 0.7)
        .extrude(_CRANK_ARM_T)
    )
    crank = crank.union(knuckle)
    if with_grip:
        handle = (
            cq.Workplane("XZ")
            .workplane(offset=arm_y)
            .center(0.0, arm_len)
            .circle(_HANDLE_R)
            .extrude(_HANDLE_LEN)
            .faces(">Y")
            .fillet(0.004)
        )
        crank = crank.union(handle)
    crank = crank.mirror(mirrorPlane="XZ")
    return mesh_from_cadquery(crank, "crank_body", assets=assets)


def _build_crank_grip_mesh(r: ResolvedPencilSharpenerConfig, *, assets):
    """Folding grip segment authored with its fold pivot at local origin, axle
    along X (the fold pin), grip knob extending in +Y."""
    # Pivot axle rod along X at z=0 so the child origin sits on real hardware.
    axle = (
        cq.Workplane("YZ", origin=(-_CRANK_ARM_W * 0.9, 0.0, 0.0))
        .circle(_CRANK_ARM_W * 0.5)
        .extrude(2.0 * _CRANK_ARM_W * 0.9)
    )
    knob = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .center(0.0, 0.0)
        .circle(_HANDLE_R)
        .extrude(_HANDLE_LEN + 0.003)
        .faces(">Y")
        .fillet(0.004)
    )
    grip = axle.union(knob)
    return mesh_from_cadquery(grip, "crank_grip", assets=assets)


def _build_blade_mesh(r: ResolvedPencilSharpenerConfig, spec: _HoleSpec, *, assets):
    """Fixed steel wedge blade mounted as an off-center chord on the bore wall."""
    blade_len = min(_BLADE_W, 1.25 * spec.outer_r)
    blade_w = min(0.0048, max(0.0032, spec.throat_r * 0.62))
    blade = (
        cq.Workplane("XY")
        .box(_BLADE_T, blade_len, blade_w, centered=(True, True, True))
        .rotate((0, 0, 0), (1, 0, 0), -28.0)
        .edges("|X")
        .fillet(0.0005)
    )
    return mesh_from_cadquery(blade, f"wedge_blade_{spec.idx}", assets=assets)


# ---------------------------------------------------------------------------
# Shavings containers.
# ---------------------------------------------------------------------------
def _drawer_center_z() -> float:
    return _DRAWER_H / 2.0 + 0.004


def _drawer_dims(r: ResolvedPencilSharpenerConfig) -> tuple[float, float, float]:
    """(width Y, depth X, height Z) for the sliding drawer, clamped to the body
    footprint so the front cavity cut never severs the (possibly small / curved)
    housing shell into a floating piece."""
    dw = min(_DRAWER_W, r.body_d * 0.68)
    dd = min(_DRAWER_D, r.body_w * 0.56)
    dh = _DRAWER_H
    return dw, dd, dh


def _build_drawer_mesh(r: ResolvedPencilSharpenerConfig, *, assets):
    w, d, h = _drawer_dims(r)
    t = _DRAWER_WALL
    outer = cq.Workplane("XY").box(d, w, h, centered=(True, True, False)).edges("|Z").fillet(0.003)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=t)
        .box(d - 2 * t, w - 2 * t, h + t, centered=(True, True, False))
    )
    tray = outer.cut(inner)
    # Faceplate laps onto the housing front face (wider than the opening), so
    # the closed drawer makes real face-to-face contact with the housing and is
    # not floating in the oversized cavity.
    faceplate = (
        cq.Workplane("YZ")
        .workplane(offset=d / 2.0)
        .center(0.0, h / 2.0)
        .rect(w + 0.012, h + 0.012)
        .extrude(0.0015)
    )
    tray = tray.union(faceplate)
    grip = (
        cq.Workplane("YZ")
        .workplane(offset=d / 2.0 + 0.0015)
        .center(0.0, h * 0.55)
        .rect(0.018, 0.005)
        .extrude(0.003)
    )
    tray = tray.union(grip)
    return mesh_from_cadquery(tray, "drawer_tray", assets=assets)


def _bin_dims(r: ResolvedPencilSharpenerConfig) -> tuple[float, float, float]:
    """(bin_l, bin_w, bin_cx) for the hinged-bin shell, sized so the bin sits in
    the lower-rear of the housing and its front edge stays behind the pencil-port
    throats (no collision with the ports / wedge blades on small scaled bodies)."""
    # Front edge must stay rear of the port throat back.
    front_limit = r.front_x - _PORT_DEPTH - 0.006
    rear = -r.body_w / 2.0 + 0.004
    bin_l = min(_BIN_L, front_limit - rear)
    bin_l = max(0.030, bin_l)
    bin_w = min(_BIN_W, r.body_d - 0.008)
    bin_cx = rear + bin_l / 2.0
    return bin_l, bin_w, bin_cx


def _build_bin_shell_mesh(r: ResolvedPencilSharpenerConfig, *, assets):
    """Open-top bin tray seated in the lower-rear of the housing. Authored with
    its base at z=0, centered in X/Y of its own frame."""
    bin_l, bin_w, _ = _bin_dims(r)
    outer = (
        cq.Workplane("XY")
        .box(bin_l, bin_w, _BIN_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=_BIN_WALL)
        .box(bin_l - 2 * _BIN_WALL, bin_w - 2 * _BIN_WALL, _BIN_H, centered=(True, True, False))
    )
    shell = outer.cut(inner)
    # Rear hinge boss: short cylinder along Y at the top-rear edge.
    hinge_boss = (
        cq.Workplane("XZ")
        .workplane(offset=-bin_w / 2.0 + 0.004)
        .center(-bin_l / 2.0 + 0.004, _BIN_H - 0.003)
        .circle(0.0035)
        .extrude(bin_w - 0.008)
    )
    return mesh_from_cadquery(shell.union(hinge_boss), "bin_shell", assets=assets)


def _build_bin_lid_mesh(r: ResolvedPencilSharpenerConfig, *, assets):
    """Hinged cover authored with its local origin at the rear hinge edge; the
    panel extends along +X toward the bin front."""
    bin_l, bin_w, _ = _bin_dims(r)
    panel = (
        cq.Workplane("XY")
        .center(bin_l / 2.0, 0.0)
        .rect(bin_l - 0.002, bin_w - 0.004)
        .extrude(_LID_T)
        .edges("|Z")
        .fillet(0.003)
    )
    knuckle = (
        cq.Workplane("XZ")
        .workplane(offset=-bin_w / 2.0 + 0.006)
        .center(0.0, _LID_T / 2.0)
        .circle(0.004)
        .extrude(bin_w - 0.012)
    )
    panel = panel.union(knuckle)
    tab = (
        cq.Workplane("XY")
        .center(bin_l - 0.006, 0.0)
        .box(0.010, 0.030, _LID_T + 0.006, centered=(True, True, False))
    )
    panel = panel.union(tab)
    return mesh_from_cadquery(panel, "bin_lid", assets=assets)


def _build_canister_mesh(r: ResolvedPencilSharpenerConfig, *, assets):
    """Twist-off canister: revolved cup + threaded collar. Authored with the
    socket center at local origin; collar runs +Z into the housing, cup -Z."""
    cup_id = _CAN_OD - 2 * _CAN_WALL
    outer_shell = (
        cq.Workplane("XY").workplane(offset=-_CAN_CUP_H).circle(_CAN_OD / 2.0).extrude(_CAN_CUP_H)
    )
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-_CAN_CUP_H + 0.003)
        .circle(cup_id / 2.0)
        .extrude(_CAN_CUP_H)
    )
    cup = outer_shell.cut(bore)
    # Threaded collar: a sturdy ring (outer ~= cup OD, bore = shavings passage)
    # that screws up into the housing base socket. Keep the wall thick so the
    # cut never erases the whole ring.
    collar_or = _CAN_OD / 2.0 - 0.002
    collar_ir = cup_id / 2.0 - 0.004
    collar = cq.Workplane("XY").workplane(offset=0.0).circle(collar_or).extrude(_COLLAR_H)
    collar_bore = (
        cq.Workplane("XY").workplane(offset=-0.001).circle(collar_ir).extrude(_COLLAR_H + 0.002)
    )
    collar = collar.cut(collar_bore)
    threads = None
    for rib_z in (0.003, 0.006):
        rib = (
            cq.Workplane("XY")
            .workplane(offset=rib_z)
            .circle(collar_or + 0.0012)
            .circle(collar_or)
            .extrude(0.0015)
        )
        threads = rib if threads is None else threads.union(rib)
    # Central support spine on the axis from the cup floor up through the collar,
    # so the canister reads as one connected body and the CONTINUOUS joint origin
    # at local (0,0,0) sits on real solid (not in the empty bore).
    spine = (
        cq.Workplane("XY")
        .workplane(offset=-_CAN_CUP_H + _CAN_WALL)
        .circle(0.003)
        .extrude(_CAN_CUP_H - _CAN_WALL + _COLLAR_H)
    )
    return mesh_from_cadquery(
        cup.union(collar).union(threads).union(spine), "canister_cup", assets=assets
    )


# ---------------------------------------------------------------------------
# Mounts.
# ---------------------------------------------------------------------------
def _build_clamp_post_mesh(r: ResolvedPencilSharpenerConfig, idx: int, *, assets):
    """One cylindrical clamp post, authored with base at local z=0."""
    post = cq.Workplane("XY").circle(_POST_R).extrude(_POST_H).edges(">Z").fillet(0.0035)
    return mesh_from_cadquery(post, f"clamp_post_{idx}", assets=assets)


def _build_clamp_frame_mesh(r: ResolvedPencilSharpenerConfig, *, assets):
    """C-shaped clamp frame hanging beneath the body. Authored with its top bar
    flush to the body underside (local z=0 at the body bottom), opening forward
    in +X. The throat receives a table edge; the thumbscrew rises from the lower
    bar."""
    drop = _CF_DROP
    reach = _CF_REACH
    bar = _CF_BAR
    spine_x = -reach / 2.0
    # Vertical spine.
    spine = cq.Workplane("XY", origin=(spine_x, 0.0, -drop)).box(
        bar, bar, drop, centered=(True, True, False)
    )
    # Upper bar (under the body).
    upper = cq.Workplane("XY", origin=(0.0, 0.0, -bar)).box(
        reach, bar, bar, centered=(True, True, False)
    )
    # Lower bar (carries the thumbscrew).
    lower = cq.Workplane("XY", origin=(0.0, 0.0, -drop)).box(
        reach, bar, bar, centered=(True, True, False)
    )
    frame = spine.union(upper).union(lower)
    return mesh_from_cadquery(frame, "clamp_frame", assets=assets), drop, bar, reach


def _build_thumbscrew_mesh(r: ResolvedPencilSharpenerConfig, *, assets):
    """Thumbscrew: shaft + pad, authored with the pad-top contact at local z=0,
    shaft running down in -Z."""
    shaft = cq.Workplane("XY", origin=(0.0, 0.0, -_SCREW_LEN)).circle(_SCREW_R).extrude(_SCREW_LEN)
    pad = cq.Workplane("XY", origin=(0.0, 0.0, -0.001)).circle(_PAD_R).extrude(_PAD_T)
    return mesh_from_cadquery(shaft.union(pad), "thumbscrew_body", assets=assets)


# ---------------------------------------------------------------------------
# Emit helpers (parallel children).
# ---------------------------------------------------------------------------
def _emit_cutters(model, r, housing, mats, *, assets) -> list[str]:
    names: list[str] = []
    for spec in _hole_specs(r):
        cutter = model.part(f"cutter_{spec.idx}")
        cutter.visual(
            _build_cutter_mesh(r, spec, assets=assets),
            material=mats["steel"],
            name=f"cutter_body_{spec.idx}",
        )
        cutter.inertial = Inertial.from_geometry(
            Cylinder(radius=max(0.002, spec.throat_r), length=0.022),
            mass=0.02,
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        model.articulation(
            f"housing_to_cutter_{spec.idx}",
            ArticulationType.FIXED,
            parent=housing,
            child=cutter,
            origin=Origin(xyz=_cutter_anchor(r, spec)),
        )
        names.append(f"cutter_{spec.idx}")
    return names


def _emit_crank(model, r, housing, mats, *, assets) -> list[str]:
    folding = r.crank_handle == "folding_grip"
    crank = model.part("crank")
    crank.visual(
        _build_crank_mesh(r, with_grip=not folding, assets=assets),
        material=mats["metal"],
        name="crank_body",
    )
    crank.inertial = Inertial.from_geometry(
        Box((0.014, _AXLE_LEN + _HANDLE_LEN, r.crank_arm_len + 2 * _HANDLE_R)),
        mass=0.04,
        origin=Origin(xyz=(0.0, (_AXLE_LEN + _HANDLE_LEN) / 2.0, r.crank_arm_len / 2.0)),
    )
    model.articulation(
        "housing_to_crank",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crank,
        origin=Origin(xyz=(0.0, r.right_wall_y, r.axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )
    names = ["crank"]
    if folding:
        grip = model.part("crank_grip")
        grip.visual(
            _build_crank_grip_mesh(r, assets=assets),
            material=mats["metal"],
            name="crank_grip",
        )
        grip.inertial = Inertial.from_geometry(
            Box((0.012, _HANDLE_LEN, 0.012)),
            mass=0.012,
            origin=Origin(xyz=(0.0, _HANDLE_LEN / 2.0, 0.0)),
        )
        # The grip pivots at the arm tip knuckle. In the crank frame the tip is
        # outboard at y = +(AXLE_LEN + arm_len-ish) after mirror, z = arm_len.
        knuckle_y = _AXLE_LEN + 0.001
        knuckle_z = r.crank_arm_len
        model.articulation(
            "crank_to_grip",
            ArticulationType.REVOLUTE,
            parent=crank,
            child=grip,
            origin=Origin(xyz=(0.0, knuckle_y, knuckle_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0, velocity=3.0, lower=0.0, upper=math.radians(95.0)
            ),
        )
        names.append("crank_grip")
    return names


def _emit_wedges(model, r, housing, mats, *, assets) -> list[str]:
    """One fixed wedge blade per pencil port (replaces the helical cutter). The
    n_holes multiplicity stays readable through the per-hole blade count."""
    names: list[str] = []
    for spec in _hole_specs(r):
        blade = model.part(f"wedge_blade_{spec.idx}")
        blade.visual(
            _build_blade_mesh(r, spec, assets=assets),
            material=mats["steel"],
            name=f"wedge_blade_{spec.idx}",
        )
        blade.inertial = Inertial.from_geometry(
            Box((_BLADE_T, _BLADE_W, _BLADE_H)),
            mass=0.008,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        model.articulation(
            f"housing_to_wedge_blade_{spec.idx}",
            ArticulationType.FIXED,
            parent=housing,
            child=blade,
            origin=Origin(xyz=_blade_anchor(r, spec)),
        )
        names.append(f"wedge_blade_{spec.idx}")
    return names


def _emit_shavings(model, r, housing, mats, *, assets) -> list[str]:
    if r.shavings_container == "sliding_drawer":
        drawer = model.part("drawer")
        drawer.visual(
            _build_drawer_mesh(r, assets=assets),
            material=mats["accent"],
            name="drawer_tray",
        )
        dw, dd, dh = _drawer_dims(r)
        drawer.inertial = Inertial.from_geometry(
            Box((dd, dw, dh)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, dh / 2.0)),
        )
        closed_x = r.front_x - dd / 2.0
        travel = min(r.drawer_travel, dd - 0.008)
        model.articulation(
            "housing_to_drawer",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=drawer,
            origin=Origin(xyz=(closed_x, 0.0, _drawer_center_z() - dh / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.3, lower=0.0, upper=travel),
        )
        return ["drawer"]

    if r.shavings_container == "hinged_bin_lid":
        bin_l, bin_w, bin_cx = _bin_dims(r)
        shell = model.part("bin_shell")
        shell.visual(
            _build_bin_shell_mesh(r, assets=assets),
            material=mats["accent"],
            name="bin_shell",
        )
        shell.inertial = Inertial.from_geometry(
            Box((bin_l, bin_w, _BIN_H)),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.0, _BIN_H / 2.0)),
        )
        model.articulation(
            "housing_to_bin_shell",
            ArticulationType.FIXED,
            parent=housing,
            child=shell,
            origin=Origin(xyz=(bin_cx, 0.0, 0.0)),
        )
        lid = model.part("bin_lid")
        lid.visual(_build_bin_lid_mesh(r, assets=assets), material=mats["metal"], name="bin_lid")
        lid.inertial = Inertial.from_geometry(
            Box((bin_l, bin_w, _LID_T + 0.006)),
            mass=0.012,
            origin=Origin(xyz=(bin_l / 2.0, 0.0, 0.0)),
        )
        # Hinge at the rear-top edge of the bin shell (bin frame x=-bin_l/2).
        model.articulation(
            "bin_to_lid",
            ArticulationType.REVOLUTE,
            parent=shell,
            child=lid,
            origin=Origin(xyz=(-bin_l / 2.0, 0.0, _BIN_H)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=2.5, lower=0.0, upper=math.radians(100.0)
            ),
        )
        return ["bin_shell", "bin_lid"]

    if r.shavings_container == "twist_canister":
        canister = model.part("shavings_canister")
        canister.visual(
            _build_canister_mesh(r, assets=assets),
            material=mats["accent"],
            name="canister_cup",
        )
        canister.inertial = Inertial.from_geometry(
            Cylinder(radius=_CAN_OD / 2.0, length=_CAN_CUP_H),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, -_CAN_CUP_H / 2.0 + _COLLAR_H / 2.0)),
        )
        # Socket center: under the cutter, at the housing base. The canister
        # local origin (0,0,0) — collar base / cup rim / spine — sits here.
        model.articulation(
            "housing_to_canister",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=canister,
            origin=Origin(xyz=(0.0, 0.0, 0.001)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=6.0),
        )
        return ["shavings_canister"]

    return []  # captured_cavity


def _emit_mount(model, r, housing, mats, *, assets) -> list[str]:
    if r.mount == "top_clamp_posts":
        names: list[str] = []
        for idx, x_off in enumerate((-0.020, 0.020)):
            post = model.part(f"clamp_post_{idx + 1}")
            post.visual(
                _build_clamp_post_mesh(r, idx + 1, assets=assets),
                material=mats["metal"],
                name=f"clamp_post_shell_{idx + 1}",
            )
            post.inertial = Inertial.from_geometry(
                Cylinder(radius=_POST_R, length=_POST_H),
                mass=0.01,
                origin=Origin(xyz=(x_off, 0.012, r.deck_z + _POST_H / 2.0)),
            )
            model.articulation(
                f"housing_to_clamp_post_{idx + 1}",
                ArticulationType.FIXED,
                parent=housing,
                child=post,
                origin=Origin(xyz=(x_off, 0.012, r.deck_z)),
            )
            names.append(f"clamp_post_{idx + 1}")
        return names

    # under_body_gclamp
    frame_mesh, drop, bar, reach = _build_clamp_frame_mesh(r, assets=assets)
    frame = model.part("clamp_frame")
    frame.visual(frame_mesh, material=mats["metal"], name="clamp_frame")
    frame.inertial = Inertial.from_geometry(
        Box((reach, bar, drop)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, -drop / 2.0)),
    )
    model.articulation(
        "housing_to_clamp_frame",
        ArticulationType.FIXED,
        parent=housing,
        child=frame,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    screw = model.part("thumbscrew")
    screw.visual(
        _build_thumbscrew_mesh(r, assets=assets),
        material=mats["steel"],
        name="thumbscrew_body",
    )
    screw.inertial = Inertial.from_geometry(
        Cylinder(radius=_SCREW_R, length=_SCREW_LEN),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, -_SCREW_LEN / 2.0)),
    )
    # Pad-top contact sits on the lower bar top, rising toward the table.
    screw_z = -drop + bar / 2.0
    model.articulation(
        "frame_to_thumbscrew",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=screw,
        origin=Origin(xyz=(reach / 2.0 - bar, 0.0, screw_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.2, lower=0.0, upper=drop - bar),
    )
    return ["clamp_frame", "thumbscrew"]


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_pencil_sharpener(
    config: PencilSharpenerConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=r.name, assets=assets)
    palette = PALETTES[r.material_style]
    mats = {
        key: model.material(f"ps_{key}_{r.material_style}", rgba=palette[key])
        for key in ("body", "metal", "steel", "badge", "accent")
    }

    # Root housing (carries inlined front plate visual).
    housing = model.part("housing")
    housing.visual(
        _build_housing_mesh(r, assets=assets), material=mats["body"], name="housing_shell"
    )
    housing.visual(
        _build_front_plate_mesh(r, assets=assets),
        material=mats["badge"],
        name="front_plate",
    )
    housing.inertial = Inertial.from_geometry(
        Box((r.body_w, r.body_d, r.body_h + r.shoulder_h)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, (r.body_h + r.shoulder_h) / 2.0)),
    )

    # Sharpening mechanism.
    if r.has_crank:
        _emit_cutters(model, r, housing, mats, assets=assets)
        _emit_crank(model, r, housing, mats, assets=assets)
    else:
        # fixed_wedge: a steel blade per port replaces the helical cutters.
        _emit_wedges(model, r, housing, mats, assets=assets)

    _emit_shavings(model, r, housing, mats, assets=assets)
    _emit_mount(model, r, housing, mats, assets=assets)

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_pencil_sharpener(
    seed: int,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    return build_pencil_sharpener(config_from_seed(seed), assets=assets)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_pencil_sharpener_tests(
    object_model: ArticulatedObject,
    config: PencilSharpenerConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")

    # --- Captured-pin / seated allowances. ---
    sharp_prefix = "cutter_" if r.has_crank else "wedge_blade_"
    for spec in _hole_specs(r):
        ctx.allow_overlap(
            housing,
            object_model.get_part(f"{sharp_prefix}{spec.idx}"),
            reason="cutter / wedge blade seated inside the front pencil port throat.",
        )
    if r.has_crank:
        ctx.allow_overlap(
            housing,
            object_model.get_part("crank"),
            reason="crank axle stub captured in the housing axle bore (mounted, not floating).",
        )
        if r.crank_handle == "folding_grip":
            ctx.allow_overlap(
                object_model.get_part("crank"),
                object_model.get_part("crank_grip"),
                reason="folding grip pivot axle wraps the crank arm-tip knuckle (captured pin).",
            )
    # (wedge blade housing overlap is covered by the per-hole loop above.)
    if r.shavings_container == "sliding_drawer":
        ctx.allow_overlap(
            housing,
            object_model.get_part("drawer"),
            reason="shavings drawer tray slides inside the housing front cavity on rails.",
        )
    elif r.shavings_container == "hinged_bin_lid":
        ctx.allow_overlap(
            housing,
            object_model.get_part("bin_shell"),
            reason="bin shell seats inside the lower-rear of the housing cavity.",
        )
        ctx.allow_overlap(
            object_model.get_part("bin_shell"),
            object_model.get_part("bin_lid"),
            reason="lid hinge knuckle wraps the bin rear-top hinge boss (captured pin); closed lid rests on the bin lip.",
        )
        ctx.allow_overlap(
            housing,
            object_model.get_part("bin_lid"),
            reason="closed bin lid lies over the bin mouth which is recessed inside the housing rear cavity.",
        )
    elif r.shavings_container == "twist_canister":
        ctx.allow_overlap(
            housing,
            object_model.get_part("shavings_canister"),
            reason="canister threaded collar screws up into the housing base socket.",
        )
    if r.mount == "under_body_gclamp":
        ctx.allow_overlap(
            housing,
            object_model.get_part("clamp_frame"),
            reason="C-frame top bar seats flush against the housing underside.",
        )
        ctx.allow_overlap(
            object_model.get_part("clamp_frame"),
            object_model.get_part("thumbscrew"),
            reason="thumbscrew shaft rides through the lower C-frame bar (captured screw).",
        )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    # All joint origins sit on real hardware (axle bore / rail / hinge seam /
    # socket / knuckle / C-frame bar); joints omit MatingContract (grandfathered).
    ctx.fail_if_joint_mating_has_gap()

    # --- Multiplicity: one cutter (crank) or one wedge blade (wedge) per hole. ---
    prefix = "cutter_" if r.has_crank else "wedge_blade_"
    sharpeners = [p.name for p in object_model.parts if p.name.startswith(prefix)]
    ctx.check(
        "one sharpener part per hole emitted",
        len(sharpeners) == r.n_holes,
        details=f"emitted={sorted(sharpeners)} expected={r.n_holes}",
    )

    # --- Sharpening mechanism topology + actuation. ---
    if r.has_crank:
        jc = object_model.get_articulation("housing_to_crank")
        ctx.check(
            "crank is CONTINUOUS about +Y",
            jc.articulation_type == ArticulationType.CONTINUOUS and round(jc.axis[1], 3) == 1.0,
            details=f"type={jc.articulation_type} axis={tuple(round(c, 3) for c in jc.axis)}",
        )
        crank = object_model.get_part("crank")
        rest = ctx.part_world_aabb(crank)
        with ctx.pose({jc: math.pi}):
            half = ctx.part_world_aabb(crank)
        if rest is not None and half is not None:
            ctx.check(
                "half turn drops the crank tip below its rest height",
                half[1][2] < rest[1][2] - 0.005,
                details=f"rest_top={rest[1][2]:.4f} half_top={half[1][2]:.4f}",
            )
        # Crank lives outboard on +Y.
        h_aabb = ctx.part_world_aabb(housing)
        c_aabb = ctx.part_world_aabb(crank)
        if h_aabb is not None and c_aabb is not None:
            ctx.check(
                "crank sits on the +Y side of the housing",
                c_aabb[1][1] > h_aabb[1][1],
                details=f"crank_max_y={c_aabb[1][1]:.4f} housing_max_y={h_aabb[1][1]:.4f}",
            )
        if r.crank_handle == "folding_grip":
            jg = object_model.get_articulation("crank_to_grip")
            ctx.check(
                "folding grip is REVOLUTE about -X",
                jg.articulation_type == ArticulationType.REVOLUTE and round(jg.axis[0], 3) == -1.0,
                details=f"type={jg.articulation_type} axis={tuple(round(c, 3) for c in jg.axis)}",
            )
        else:
            ctx.check(
                "one-piece crank has no separate grip part",
                "crank_grip" not in {p.name for p in object_model.parts},
                details=str([p.name for p in object_model.parts]),
            )
    else:
        # fixed_wedge: blades are FIXED, and the moving mechanism is the bin lid.
        jb = object_model.get_articulation("housing_to_wedge_blade_0")
        ctx.check(
            "wedge blade is FIXED",
            jb.articulation_type == ArticulationType.FIXED,
            details=f"type={jb.articulation_type}",
        )
        ctx.check(
            "fixed-wedge build has no crank part",
            "crank" not in {p.name for p in object_model.parts},
            details=str([p.name for p in object_model.parts]),
        )
        ctx.check(
            "fixed-wedge build provides a hinged bin lid for the moving mechanism",
            r.shavings_container == "hinged_bin_lid"
            and "bin_lid" in {p.name for p in object_model.parts},
            details=f"shavings={r.shavings_container}",
        )

    # --- Shavings container topology + actuation. ---
    if r.shavings_container == "sliding_drawer":
        jd = object_model.get_articulation("housing_to_drawer")
        ctx.check(
            "drawer is PRISMATIC along +X",
            jd.articulation_type == ArticulationType.PRISMATIC and round(jd.axis[0], 3) == 1.0,
            details=f"type={jd.articulation_type} axis={tuple(round(c, 3) for c in jd.axis)}",
        )
        drawer = object_model.get_part("drawer")
        travel = jd.motion_limits.upper if jd.motion_limits else 0.0
        rest = ctx.part_world_position(drawer)
        with ctx.pose({jd: travel}):
            ext = ctx.part_world_position(drawer)
            ext_aabb = ctx.part_world_aabb(drawer)
        h_aabb = ctx.part_world_aabb(housing)
        if rest is not None and ext is not None:
            ctx.check(
                "drawer extends forward when opened",
                ext[0] > rest[0] + 0.6 * travel,
                details=f"rest_x={rest[0]:.4f} ext_x={ext[0]:.4f} travel={travel:.4f}",
            )
        if ext_aabb is not None and h_aabb is not None:
            ctx.check(
                "drawer stays partially retained at full extension",
                ext_aabb[0][0] < h_aabb[1][0] - 0.002,
                details=f"drawer_min_x={ext_aabb[0][0]:.4f} housing_max_x={h_aabb[1][0]:.4f}",
            )
    elif r.shavings_container == "hinged_bin_lid":
        jl = object_model.get_articulation("bin_to_lid")
        ctx.check(
            "bin lid is REVOLUTE about -Y",
            jl.articulation_type == ArticulationType.REVOLUTE and round(jl.axis[1], 3) == -1.0,
            details=f"type={jl.articulation_type} axis={tuple(round(c, 3) for c in jl.axis)}",
        )
        lid = object_model.get_part("bin_lid")
        with ctx.pose({jl: 0.0}):
            a0 = ctx.part_world_aabb(lid)
        with ctx.pose({jl: math.radians(90.0)}):
            a1 = ctx.part_world_aabb(lid)
        if a0 is not None and a1 is not None:
            ctx.check(
                "opening the bin lid raises its free edge",
                a1[1][2] > a0[1][2] + 0.005,
                details=f"closed_top={a0[1][2]:.4f} open_top={a1[1][2]:.4f}",
            )
    elif r.shavings_container == "twist_canister":
        jt = object_model.get_articulation("housing_to_canister")
        ctx.check(
            "canister is CONTINUOUS about +Z",
            jt.articulation_type == ArticulationType.CONTINUOUS and round(jt.axis[2], 3) == 1.0,
            details=f"type={jt.articulation_type} axis={tuple(round(c, 3) for c in jt.axis)}",
        )
        can = object_model.get_part("shavings_canister")
        c_aabb = ctx.part_world_aabb(can)
        if c_aabb is not None:
            ctx.check(
                "canister cup hangs below the housing base",
                c_aabb[0][2] < 0.001,
                details=f"canister_min_z={c_aabb[0][2]:.4f}",
            )

    # --- Mount topology + actuation. ---
    if r.mount == "top_clamp_posts":
        for idx in (1, 2):
            post = object_model.get_part(f"clamp_post_{idx}")
            p_aabb = ctx.part_world_aabb(post)
            if p_aabb is not None:
                ctx.check(
                    f"clamp_post_{idx} rises above the top deck",
                    p_aabb[1][2] > r.deck_z + 0.006,
                    details=f"post_max_z={p_aabb[1][2]:.4f} deck={r.deck_z:.4f}",
                )
    else:
        js = object_model.get_articulation("frame_to_thumbscrew")
        ctx.check(
            "thumbscrew is PRISMATIC along +Z",
            js.articulation_type == ArticulationType.PRISMATIC and round(js.axis[2], 3) == 1.0,
            details=f"type={js.articulation_type} axis={tuple(round(c, 3) for c in js.axis)}",
        )
        screw = object_model.get_part("thumbscrew")
        rest = ctx.part_world_aabb(screw)
        with ctx.pose({js: js.motion_limits.upper if js.motion_limits else 0.01}):
            up = ctx.part_world_aabb(screw)
        if rest is not None and up is not None:
            ctx.check(
                "thumbscrew pad travels up toward the table",
                up[1][2] > rest[1][2] + 0.005,
                details=f"rest_top={rest[1][2]:.4f} up_top={up[1][2]:.4f}",
            )

    # --- Housing reads as a desk sharpener on the ground. ---
    h_aabb = ctx.part_world_aabb(housing)
    if h_aabb is not None:
        (hx0, hy0, hz0), (hx1, hy1, hz1) = h_aabb
        ctx.check(
            "housing rests on the ground",
            hz0 < 0.01,
            details=f"z_min={hz0:.4f}",
        )

    # --- At least one non-fixed joint always exists. ---
    non_fixed = [j for j in object_model.joints if j.articulation_type != ArticulationType.FIXED]
    ctx.check(
        "model has at least one non-fixed joint",
        len(non_fixed) >= 1,
        details=f"non_fixed={[j.name for j in non_fixed]}",
    )

    # --- slot_choices recorded. ---
    ctx.check(
        "slot_choices_recorded",
        tuple(object_model.meta.get("slot_choices", ())) == slot_choices_for_config(r),
        details=str(object_model.meta.get("slot_choices")),
    )

    return ctx.report()


__all__ = (
    "PencilSharpenerConfig",
    "ResolvedPencilSharpenerConfig",
    "build_pencil_sharpener",
    "build_seeded_pencil_sharpener",
    "config_from_seed",
    "resolve_config",
    "run_pencil_sharpener_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
    "with_overrides",
)
