"""Red painted sheet-metal fire bucket / fire pail — modular template.

Sourced from the reviewed modular spec
``articraft_template_authoring/specs_modular_v1/Urban_Environment_Urban_Environment_bucket1.md`` and the
``picture/.../bucket1`` 5-star sample pool (2 parents + 9 single-axis variants).

Core identity (spec §核心身份): a red painted thin-wall sheet-metal fire bucket —
(1) a hollow **revolved thin-wall shell** body (conical-pointed / tapered /
straight / hemispherical / deep-narrow-cone), (2) a **rolled rim** torus at the
mouth (identity invariant, never removed), (3) ±Y riveted **pivot lugs**, and
(4) a steel-wire **BAIL handle** swinging on a **REVOLUTE** joint about the ±Y
lug diameter line — the bail-swing REVOLUTE is the *defining joint*. Each variant
keeps it, or replaces it with an equivalent real non-FIXED joint (a fold-flat
side grip, or a hinged lid). Optional wall bracket / hook ring mounting.

Pattern = ``parallel_children`` + one body-local reinforcing-band ``multiplicity``
axis. Four module slots:

  * ``body_profile`` (5): conical_pointed / tapered_cylinder / straight_pail /
    hemispherical_bowl / deep_narrow_cone. Drives the ``_radius_at(z)`` revolve
    profile of the root body. The body is always a ``LatheGeometry``
    surface-of-revolution (Rule 3), never a Box / Cylinder downgrade.
  * ``handle`` (3, DEFINING JOINT): swing_bail_revolute (axis ±Y) /
    fixed_side_grips (one fold-flat REVOLUTE axis ±X + one inline grip) /
    hinged_lid (lid REVOLUTE axis ±Y; deletes lugs/bail).
  * ``mount`` (3): free_standing / wall_bracket (root=bracket, FIXED reparent) /
    hook_ring (inline decorative hanging ring).
  * ``band_count`` (multiplicity): N reinforcing hoop ribs, N ∈ {0,2,3,4,5},
    each a torus at the local wall radius. N=0 == rolled-rim-only.

Three hard rules (spec §核心 + §Rules):
  * Rule 1 — bands / rim / lugs / rivets / hook ring decorations are non-moving
    ``parent.visual(...)`` on the body, never FIXED-jointed parts.
  * Rule 2 — the defining bail (and every real non-FIXED joint) declares a
    ``MatingContract`` and its origin sits on the real lug pivot axis.
  * Rule 3 — the body is a true surface-of-revolution (``LatheGeometry``).

Compatibility gating (resolve_config, spec §compatibility matrix):
  * ``hinged_lid`` deletes lugs/rivets/bail (no double-handle semantics); the lid
    is the defining joint. Compatible with every mount.
  * Cone family (conical_pointed / deep_narrow_cone) has no flat bottom: it hangs
    from the bail / mount rather than standing; tests assert "apex lowest" not
    "flat bottom". A cone with no real non-FIXED joint (hinged_lid removed lugs,
    but cone+lid is allowed) is still fine — lid provides the joint.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, replace
from typing import Literal

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MatingContract,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

__modular__ = True

BodyProfile = Literal[
    "conical_pointed",
    "tapered_cylinder",
    "straight_pail",
    "hemispherical_bowl",
    "deep_narrow_cone",
]
Handle = Literal[
    "swing_bail_revolute",
    "fixed_side_grips",
    "hinged_lid",
]
Mount = Literal[
    "free_standing",
    "wall_bracket",
    "hook_ring",
]
PaletteStyle = Literal[
    "fire_red",
    "galvanized",
    "sand_tan",
    "weathered_brick",
    "hammered_gunmetal",
    "forest_green",
]

BODY_PROFILES: tuple[BodyProfile, ...] = (
    "conical_pointed",
    "tapered_cylinder",
    "straight_pail",
    "hemispherical_bowl",
    "deep_narrow_cone",
)
HANDLES: tuple[Handle, ...] = (
    "swing_bail_revolute",
    "fixed_side_grips",
    "hinged_lid",
)
MOUNTS: tuple[Mount, ...] = (
    "free_standing",
    "wall_bracket",
    "hook_ring",
)
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "fire_red",
    "galvanized",
    "sand_tan",
    "weathered_brick",
    "hammered_gunmetal",
    "forest_green",
)

# Cone family: apex-down, no flat bottom (hangs, doesn't stand).
_CONE_FAMILY: tuple[BodyProfile, ...] = ("conical_pointed", "deep_narrow_cone")

# Sampling weights (spec §Procedural Sampling).
_BODY_WEIGHTS = (0.20, 0.24, 0.22, 0.18, 0.16)
_HANDLE_WEIGHTS = (0.60, 0.25, 0.15)
_MOUNT_WEIGHTS = (0.50, 0.20, 0.30)  # free / bracket / hook
# Band-count multiplicity domain (spec §Multiplicity weight bands).
_BAND_N = ((0, 2, 3, 4, 5), (0.40, 0.30, 0.20, 0.07, 0.03))

PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "fire_red": {
        "body": (0.62, 0.09, 0.08, 1.0),
        "rim": (0.50, 0.07, 0.06, 1.0),
        "wire": (0.72, 0.74, 0.77, 1.0),
        "lug": (0.45, 0.06, 0.05, 1.0),
        "accent": (0.80, 0.80, 0.82, 1.0),
    },
    "galvanized": {
        "body": (0.70, 0.72, 0.74, 1.0),
        "rim": (0.60, 0.62, 0.64, 1.0),
        "wire": (0.50, 0.52, 0.55, 1.0),
        "lug": (0.55, 0.57, 0.60, 1.0),
        "accent": (0.64, 0.66, 0.68, 1.0),
    },
    "sand_tan": {
        "body": (0.78, 0.68, 0.46, 1.0),
        "rim": (0.66, 0.57, 0.38, 1.0),
        "wire": (0.40, 0.38, 0.34, 1.0),
        "lug": (0.58, 0.50, 0.34, 1.0),
        "accent": (0.72, 0.62, 0.42, 1.0),
    },
    "weathered_brick": {
        "body": (0.52, 0.24, 0.18, 1.0),
        "rim": (0.42, 0.19, 0.14, 1.0),
        "wire": (0.46, 0.44, 0.42, 1.0),
        "lug": (0.38, 0.17, 0.12, 1.0),
        "accent": (0.60, 0.30, 0.22, 1.0),
    },
    "hammered_gunmetal": {
        "body": (0.30, 0.32, 0.35, 1.0),
        "rim": (0.24, 0.26, 0.29, 1.0),
        "wire": (0.42, 0.44, 0.47, 1.0),
        "lug": (0.22, 0.24, 0.27, 1.0),
        "accent": (0.38, 0.40, 0.43, 1.0),
    },
    "forest_green": {
        "body": (0.18, 0.34, 0.22, 1.0),
        "rim": (0.14, 0.28, 0.18, 1.0),
        "wire": (0.50, 0.52, 0.50, 1.0),
        "lug": (0.13, 0.25, 0.16, 1.0),
        "accent": (0.24, 0.42, 0.28, 1.0),
    },
}

# ---------------------------------------------------------------------------
# Base real-world dimensions (meters). Vertical axis +Z; mouth at z=BODY_H.
# A hand-carried fire bucket: ~0.25-0.40 m tall, ~0.16-0.28 m mouth diameter.
# ---------------------------------------------------------------------------
_TOP_R = 0.135  # mouth (rim) outer radius
_BODY_H = 0.300  # body outer height (rim plane)
_WALL = 0.010  # wall thickness
_FLOOR_T = 0.012  # solid floor disc thickness (lathe/straight families)
_RIM_TUBE = 0.011  # rolled rim torus tube radius
_BAND_TUBE = 0.0075  # reinforcing band torus tube radius

_LUG_Z = 0.288  # ~ BODY_H - 0.012; pivot lug center height
_LUG_W = 0.030  # lug pad size
_LUG_T = 0.014  # lug pad radial thickness
_LUG_Y_OFF = 0.014  # lug center radial offset past the wall

_WIRE_R = 0.006  # bail / grip steel wire radius
_HANDLE_RISE = 0.150  # bail arch rise above the pivot line

_LATHE_SEGMENTS = 56  # revolve angular segments


@dataclass(frozen=True)
class Bucket1Config:
    body_profile: BodyProfile | None = None
    handle: Handle | None = None
    mount: Mount | None = None
    band_count: int | None = None
    palette_style: PaletteStyle | None = None
    body_height_scale: float = 1.0
    top_radius_scale: float = 1.0
    taper_ratio: float = 0.75
    handle_rise_scale: float = 1.0
    cradle_z_frac: float = 0.38
    name: str = "bucket1"


@dataclass(frozen=True)
class ResolvedBucket1Config:
    body_profile: BodyProfile
    handle: Handle
    mount: Mount
    band_count: int
    palette_style: PaletteStyle
    # Concrete geometry.
    top_r: float
    bot_r: float
    body_h: float
    wall: float
    floor_t: float
    rim_tube: float
    band_tube: float
    lug_z: float
    lug_y_off: float
    handle_rise: float
    cradle_z_frac: float
    name: str

    @property
    def is_cone(self) -> bool:
        return self.body_profile in _CONE_FAMILY

    @property
    def rim_outer_y(self) -> float:
        """Outer Y extent of the rolled rim at the mouth."""
        return self.top_r + self.rim_tube

    @property
    def lug_y(self) -> float:
        """Pivot-lug center radius on the ±Y diameter line."""
        return self.top_r + self.lug_y_off


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _pick(value, choices):
    return value if value in choices else choices[0]


def config_from_seed(seed: int) -> Bucket1Config:
    rng = random.Random(seed)
    body_profile = rng.choices(BODY_PROFILES, weights=_BODY_WEIGHTS, k=1)[0]
    handle = rng.choices(HANDLES, weights=_HANDLE_WEIGHTS, k=1)[0]
    mount = rng.choices(MOUNTS, weights=_MOUNT_WEIGHTS, k=1)[0]
    band_count = rng.choices(_BAND_N[0], weights=_BAND_N[1], k=1)[0]
    return Bucket1Config(
        body_profile=body_profile,
        handle=handle,
        mount=mount,
        band_count=band_count,
        palette_style=rng.choice(PALETTE_STYLES),
        body_height_scale=round(rng.uniform(0.85, 1.20), 4),
        top_radius_scale=round(rng.uniform(0.90, 1.15), 4),
        taper_ratio=round(rng.uniform(0.55, 1.00), 4),
        handle_rise_scale=round(rng.uniform(0.85, 1.25), 4),
        cradle_z_frac=round(rng.uniform(0.30, 0.45), 4),
        name=f"seeded_bucket1_{seed}",
    )


def resolve_config(config: Bucket1Config | None = None) -> ResolvedBucket1Config:
    cfg = config or Bucket1Config()
    palette_style = _pick(cfg.palette_style, PALETTE_STYLES)
    body_profile = _pick(cfg.body_profile, BODY_PROFILES)
    handle = _pick(cfg.handle, HANDLES)
    mount = _pick(cfg.mount, MOUNTS)

    # --- Continuous scales (clamped). ---
    body_h = _clamp(_BODY_H * _clamp(cfg.body_height_scale, 0.85, 1.20), 0.240, 0.400)
    top_r = _clamp(_TOP_R * _clamp(cfg.top_radius_scale, 0.90, 1.15), 0.100, 0.165)
    taper_ratio = _clamp(cfg.taper_ratio, 0.55, 1.00)
    handle_rise = _clamp(_HANDLE_RISE * _clamp(cfg.handle_rise_scale, 0.85, 1.25), 0.100, 0.220)
    cradle_z_frac = _clamp(cfg.cradle_z_frac, 0.30, 0.45)

    # --- body_profile -> bot_r (equation: BOT_R = taper_ratio * TOP_R). ---
    if body_profile == "straight_pail":
        bot_r = top_r  # straight wall: top == bot
    elif body_profile == "conical_pointed":
        bot_r = 0.0  # apex-down point
    elif body_profile == "deep_narrow_cone":
        bot_r = 0.0
        # deeper, narrower funnel: taller + slightly narrower mouth.
        body_h = _clamp(body_h * 1.20, 0.240, 0.460)
        top_r = _clamp(top_r * 0.80, 0.090, 0.150)
    elif body_profile == "hemispherical_bowl":
        # small flat bottom (a quarter-arc bowl), shorter than a pail.
        bot_r = _clamp(top_r * 0.20, 0.018, 0.05)
        body_h = _clamp(body_h * 0.85, 0.200, 0.360)
    else:  # tapered_cylinder: foot narrower than mouth
        bot_r = _clamp(taper_ratio * top_r, 0.045, top_r - 0.010)

    # --- bail-rim clearance inequality: the bail linkage must stand outside the
    #     rim (LUG_Y - WIRE_R > rim_outer_y + 0.001). Widen the lug offset if not. ---
    lug_y_off = _LUG_Y_OFF
    for _ in range(8):
        lug_y = top_r + lug_y_off
        if (lug_y - _WIRE_R) > (top_r + _RIM_TUBE) + 0.001:
            break
        lug_y_off += 0.004

    lug_z = _clamp(_LUG_Z * (body_h / _BODY_H), body_h - 0.060, body_h - 0.006)

    return ResolvedBucket1Config(
        body_profile=body_profile,
        handle=handle,
        mount=mount,
        band_count=int(_clamp(int(cfg.band_count or 0), 0, 5)),
        palette_style=palette_style,
        top_r=top_r,
        bot_r=bot_r,
        body_h=body_h,
        wall=_WALL,
        floor_t=_FLOOR_T,
        rim_tube=_RIM_TUBE,
        band_tube=_BAND_TUBE,
        lug_z=lug_z,
        lug_y_off=lug_y_off,
        handle_rise=handle_rise,
        cradle_z_frac=cradle_z_frac,
        name=cfg.name or "bucket1",
    )


def with_overrides(config: Bucket1Config, **kwargs: object) -> Bucket1Config:
    return replace(config, **kwargs)


# ---------------------------------------------------------------------------
# Slot A: body revolve profile. _radius_at(z) is the single switch point.
# ---------------------------------------------------------------------------
def _radius_at(r: ResolvedBucket1Config, z: float) -> float:
    """Outer body radius at height z in [0, body_h]."""
    h = max(r.body_h, 1e-6)
    t = _clamp(z / h, 0.0, 1.0)
    if r.body_profile == "straight_pail":
        return r.top_r
    if r.body_profile in _CONE_FAMILY:
        # apex (r=0) at z=0 -> top_r at z=body_h, linear.
        return r.bot_r + (r.top_r - r.bot_r) * t
    if r.body_profile == "hemispherical_bowl":
        # quarter-arc bowl: r grows non-linearly (curvature) from bot_r to top_r.
        # r(t) = bot_r + (top_r - bot_r) * sin(pi/2 * t)
        return r.bot_r + (r.top_r - r.bot_r) * math.sin(0.5 * math.pi * t)
    # tapered_cylinder: linear, wider at top.
    return r.bot_r + (r.top_r - r.bot_r) * t


# ---------------------------------------------------------------------------
# Slot A body shell: always a LatheGeometry surface-of-revolution (Rule 3).
# ---------------------------------------------------------------------------
def _body_profile_points(r: ResolvedBucket1Config) -> list[tuple[float, float]]:
    """(radius, z) profile for the hollow lathe body: up the outer wall, across
    the rim, down the inner wall, across the floor top, to the axis. Cone-family
    buckets start at an apex point (r=0) with no flat floor."""
    n = 16
    floor_top = 0.0 if r.is_cone else r.floor_t
    outer: list[tuple[float, float]] = []
    z_lo = 0.0
    for i in range(n + 1):
        z = z_lo + (r.body_h - z_lo) * i / n
        outer.append((_radius_at(r, z), z))
    rin_top = max(0.012, _radius_at(r, r.body_h) - r.wall)
    inner: list[tuple[float, float]] = []
    for i in range(n + 1):
        z = r.body_h - (r.body_h - floor_top) * i / n
        ro = _radius_at(r, z)
        inner.append((max(0.008, ro - r.wall), z))

    pts: list[tuple[float, float]] = []
    if r.is_cone:
        pts.append((0.0, 0.0))  # apex point
        pts += outer[1:]  # up the outer wall from apex
        pts.append((rin_top, r.body_h))  # across the rim
        pts += inner  # down inner wall back to apex region
        pts.append((0.0, max(0.0, r.wall)))  # close near apex on axis
    else:
        pts.append((0.0, 0.0))  # floor center bottom
        pts += outer  # up outer wall
        pts.append((rin_top, r.body_h))  # across the rim
        pts += inner  # down inner wall
        pts.append((0.0, floor_top))  # across the floor top to axis
    return pts


def _emit_body_shell(body, r: ResolvedBucket1Config, mats) -> None:
    """The root revolved thin-wall shell (single hollow lathe). Rule 3."""
    lathe = LatheGeometry(_body_profile_points(r), segments=_LATHE_SEGMENTS, closed=True)
    body.visual(
        mesh_from_geometry(lathe, "bucket_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mats["body"],
        name="bucket_shell",
    )


def _emit_rolled_rim(body, r: ResolvedBucket1Config, mats) -> None:
    """Rolled top rim torus (identity invariant, Rule 1) + floor disc."""
    rim = TorusGeometry(
        r.top_r, r.rim_tube, radial_segments=12, tubular_segments=_LATHE_SEGMENTS
    )
    body.visual(
        mesh_from_geometry(rim, "rolled_rim"),
        origin=Origin(xyz=(0.0, 0.0, r.body_h - r.rim_tube * 0.4)),
        material=mats["rim"],
        name="rolled_rim",
    )
    if not r.is_cone:
        body.visual(
            Cylinder(radius=max(0.012, _radius_at(r, r.floor_t * 0.5) - 0.001), length=r.floor_t),
            origin=Origin(xyz=(0.0, 0.0, r.floor_t * 0.5)),
            material=mats["body"],
            name="floor_disc",
        )


# ---------------------------------------------------------------------------
# Slot D: reinforcing band multiplicity. N hoop torus ribs hugging the wall.
# ---------------------------------------------------------------------------
def _band_heights(r: ResolvedBucket1Config) -> list[float]:
    """Evenly distributed band z-heights, margin off the floor and rim."""
    n = r.band_count
    if n <= 0:
        return []
    z_lo = r.body_h * 0.08
    z_hi = r.body_h * 0.92
    span = z_hi - z_lo
    return [z_lo + span * (i + 1) / (n + 1) for i in range(n)]


def _emit_bands(body, r: ResolvedBucket1Config, mats) -> None:
    """N reinforcing hoop ribs; each torus radius = local wall radius (Rule 1)."""
    for i, z in enumerate(_band_heights(r)):
        ro = _radius_at(r, z)
        band = TorusGeometry(
            max(0.012, ro - r.band_tube * 0.3),
            r.band_tube,
            radial_segments=10,
            tubular_segments=48,
        )
        body.visual(
            mesh_from_geometry(band, f"band_{i}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=mats["accent"],
            name=f"band_{i}",
        )


# ---------------------------------------------------------------------------
# Pivot lug pads on the ±Y diameter line (carry-handle anchors, Rule 1).
# ---------------------------------------------------------------------------
_SIDE_SIGNS = (1.0, -1.0)
_SIDE_TAGS = ("pos", "neg")


def _emit_pivot_lug(body, r: ResolvedBucket1Config, mats, *, sgn: float, z: float, wall_r: float, name: str) -> None:
    """A riveted pivot lug standoff bracket on the ±Y diameter line. It bridges
    radially from the wall surface (embedded, no island) out to the pivot line
    (lug_y), so the wire seats on its outer face clear of the rim."""
    ly = r.lug_y
    inner = wall_r - 0.006  # embed the inner end into the shell wall
    outer = ly + _LUG_T * 0.5  # outer pivot face (just past the pivot line)
    span = outer - inner
    center = 0.5 * (inner + outer)
    body.visual(
        Box((_LUG_W, span, _LUG_W)),
        origin=Origin(xyz=(0.0, sgn * center, z)),
        material=mats["lug"],
        name=name,
    )
    body.visual(
        Cylinder(radius=0.005, length=span * 0.9),
        origin=Origin(xyz=(0.0, sgn * center, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["accent"],
        name=f"rivet_{name.split('_')[-1]}",
    )


# ---------------------------------------------------------------------------
# Slot B: handle. (a) swing bail REVOLUTE, (b) fold-flat side grips, (c) lid.
# ---------------------------------------------------------------------------
def _emit_swing_bail(model, r: ResolvedBucket1Config, body, mats) -> list[str]:
    """Single steel-wire BAIL handle on a REVOLUTE joint about the ±Y lug axis.
    DEFINING JOINT: origin (0,0,lug_z), axis (0,1,0), limits ±100 deg."""
    wall_top_r = _radius_at(r, r.lug_z)
    for sgn, tag in zip(_SIDE_SIGNS, _SIDE_TAGS):
        _emit_pivot_lug(body, r, mats, sgn=sgn, z=r.lug_z, wall_r=wall_top_r, name=f"lug_{tag}")
    # Pivot cross-axle on the body: a thin rod spanning the two lugs along Y at
    # the rim line (z=lug_z). It is the bail's pivot pin and anchors the REVOLUTE
    # joint origin (0,0,lug_z) on real body geometry (the cavity is empty there).
    ly = r.lug_y
    body.visual(
        Cylinder(radius=0.004, length=2.0 * ly),
        origin=Origin(xyz=(0.0, 0.0, r.lug_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["accent"],
        name="bail_axle",
    )

    handle = model.part("handle")
    rise = r.handle_rise
    # Authored in the PIVOT-LOCAL frame: the child link sits at the joint origin
    # (0,0,lug_z), so geometry is relative to z=0 there. The bail arches up from
    # the +Y lug (y=+ly,z=0), over the mouth, down to the -Y lug. q=0 upright.
    apex_z = rise
    bail = tube_from_spline_points(
        [
            (0.0, ly, 0.0),
            (0.0, ly * 0.95, rise * 0.55),
            (0.0, ly * 0.5, apex_z - rise * 0.10),
            (0.0, 0.0, apex_z),
            (0.0, -ly * 0.5, apex_z - rise * 0.10),
            (0.0, -ly * 0.95, rise * 0.55),
            (0.0, -ly, 0.0),
        ],
        radius=_WIRE_R,
        samples_per_segment=12,
        radial_segments=8,
        cap_ends=True,
    )
    handle.visual(mesh_from_geometry(bail, "bail_wire"), material=mats["wire"], name="bail_wire")
    # Child pivot axle along Y at z=0 (the bail's own pivot pin), placing real
    # geometry at the child link origin (0,0,0) where the REVOLUTE joint sits.
    handle.visual(
        Cylinder(radius=0.005, length=2.0 * ly),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["wire"],
        name="bail_pin",
    )
    # End seat pads straddling the two pivot lugs (child-frame contact faces).
    # Centered on the lug center so the seat envelops the lug pad; its negative_y
    # face meets the lug positive_y face within tol.
    for sgn, tag in zip(_SIDE_SIGNS, _SIDE_TAGS):
        handle.visual(
            Box((_LUG_W * 0.8, 0.030, _LUG_W * 0.8)),
            origin=Origin(xyz=(0.0, sgn * ly, 0.0)),
            material=mats["wire"],
            name=f"bail_end_{tag}",
        )
    handle.inertial = Inertial.from_geometry(
        Box((2.0 * _WIRE_R, 2.0 * ly, rise)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, rise * 0.4)),
    )
    model.articulation(
        "bucket_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, r.lug_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.5, lower=-math.radians(100), upper=math.radians(100)
        ),
        mating=MatingContract(
            parent_face_geometry="lug_pos",
            parent_face_side="positive_y",
            child_face_geometry="bail_end_pos",
            child_face_side="positive_y",
            contact_tol=0.012,
        ),
    )
    return ["handle"]


def _emit_side_grips(model, r: ResolvedBucket1Config, body, mats) -> list[str]:
    """Two ±Y D-loop side grips. The +Y grip is a fold-flat REVOLUTE child part
    (axis ±X, 0..85 deg, the real non-FIXED joint); the -Y grip is an inline
    fixed grip visual on the body."""
    grip_z = _clamp(r.body_h - 0.055, 0.05, r.body_h - 0.02)
    wall_grip_r = _radius_at(r, grip_z)
    gy = wall_grip_r + r.lug_y_off
    names: list[str] = []
    # Pivot lugs at both sides: standoff brackets embedded into the wall (no
    # island) bridging out to the grip pivot line.
    g_inner = wall_grip_r - 0.006
    g_outer = gy + _LUG_T * 0.5
    g_span = g_outer - g_inner
    g_center = 0.5 * (g_inner + g_outer)
    for sgn, tag in zip(_SIDE_SIGNS, _SIDE_TAGS):
        body.visual(
            Box((_LUG_W, g_span, _LUG_W)),
            origin=Origin(xyz=(0.0, sgn * g_center, grip_z)),
            material=mats["lug"],
            name=f"grip_lug_{tag}",
        )

    # --- fold-flat REVOLUTE grip (+Y side). ---
    fold = model.part("fold_grip")
    # D-loop authored about the pivot at part-local origin (the +Y lug), reaching
    # outward in +Y and arching down; rotates about X.
    loop = tube_from_spline_points(
        [
            (-0.022, 0.004, 0.0),
            (-0.026, 0.040, 0.0),
            (0.0, 0.052, 0.0),
            (0.026, 0.040, 0.0),
            (0.022, 0.004, 0.0),
        ],
        radius=_WIRE_R,
        samples_per_segment=10,
        radial_segments=8,
        cap_ends=True,
    )
    fold.visual(mesh_from_geometry(loop, "grip_wire_0"), material=mats["wire"], name="grip_wire_0")
    # Anchor bar spanning the two wire feet (x=±0.022, y≈0.004) so it envelops
    # both ends — no island — and its negative_y face seats on the pivot lug.
    fold.visual(
        Box((0.060, 0.024, _LUG_W * 0.7)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=mats["wire"],
        name="grip_anchor_0",
    )
    fold.inertial = Inertial.from_geometry(
        Box((0.06, 0.06, 0.03)), mass=0.05, origin=Origin(xyz=(0.0, 0.02, 0.0))
    )
    model.articulation(
        "bucket_to_fold_grip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=fold,
        origin=Origin(xyz=(0.0, gy, grip_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5, velocity=2.0, lower=0.0, upper=math.radians(85)
        ),
        mating=MatingContract(
            parent_face_geometry="grip_lug_pos",
            parent_face_side="positive_y",
            child_face_geometry="grip_anchor_0",
            child_face_side="negative_y",
            contact_tol=0.014,
        ),
    )
    names.append("fold_grip")

    # --- inline fixed grip (-Y side): a body visual, not a part. Its feet seat
    # on grip_lug_neg (whose outer face is at y = -g_outer) so it is not an
    # island; the loop arches outward in -Y. ---
    foot_y = -g_outer + 0.004  # just inside the lug outer face (embedded)
    fixed_loop = tube_from_spline_points(
        [
            (-0.012, foot_y, grip_z),
            (-0.026, foot_y - 0.036, grip_z),
            (0.0, foot_y - 0.048, grip_z),
            (0.026, foot_y - 0.036, grip_z),
            (0.012, foot_y, grip_z),
        ],
        radius=_WIRE_R,
        samples_per_segment=10,
        radial_segments=8,
        cap_ends=True,
    )
    body.visual(
        mesh_from_geometry(fixed_loop, "grip_wire_1"),
        material=mats["wire"],
        name="grip_wire_1",
    )
    return names


def _lid_disc_mesh(r: ResolvedBucket1Config, name: str):
    """Flat lid disc as a lathe surface-of-revolution. Authored lid-local with
    z=0 at the underside; disc + small rim."""
    rd = r.top_r + 0.010
    t = 0.012
    pts = [
        (0.0, 0.0),
        (rd, 0.0),
        (rd, t),
        (rd - 0.008, t + 0.006),  # small upturned rim
        (0.0, t),
    ]
    return mesh_from_geometry(LatheGeometry(pts, segments=_LATHE_SEGMENTS, closed=True), name)


def _emit_hinged_lid(model, r: ResolvedBucket1Config, body, mats) -> list[str]:
    """Hinged flat lid: a REVOLUTE about a rim-tangent +X hinge line (axis ±Y),
    0..110 deg. Replaces lugs/bail (the lid is the defining joint)."""
    hinge_x = r.top_r
    rim_z = r.body_h
    # Hinge ears + barrel on the +X rim (real anchoring visuals).
    for k, sy in ((0, 1.0), (1, -1.0)):
        body.visual(
            Box((0.018, 0.012, 0.016)),
            origin=Origin(xyz=(hinge_x - 0.006, sy * 0.020, rim_z - 0.004)),
            material=mats["lug"],
            name=f"hinge_ear_{k}",
        )
    body.visual(
        Cylinder(radius=0.008, length=0.052),
        origin=Origin(xyz=(hinge_x - 0.004, 0.0, rim_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["lug"],
        name="hinge_barrel",
    )

    lid = model.part("lid")
    # Lid authored in the hinge pivot frame: pivot at part-local origin; disc
    # center sits above the rim, shifted by -hinge_x in X. Lid seats above rim.
    seat_dz = max(r.rim_tube + 0.002, 0.013)
    lid.visual(
        _lid_disc_mesh(r, "lid_disk"),
        origin=Origin(xyz=(-hinge_x, 0.0, seat_dz)),
        material=mats["accent"],
        name="lid_disk",
    )
    # Strap bridging the hinge barrel (child origin, z=0) up to the lid disk
    # underside (z=seat_dz). Spans X under the disk inner edge and Z across the
    # seat gap so it touches both — no island, and lands at the child origin.
    strap_h = seat_dz + 0.004
    lid.visual(
        Box((0.030, 0.014, strap_h)),
        origin=Origin(xyz=(-0.008, 0.0, strap_h * 0.5)),
        material=mats["lug"],
        name="hinge_strap",
    )
    # Knob on the lid top.
    lid.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(-hinge_x, 0.0, seat_dz + 0.018)),
        material=mats["wire"],
        name="lid_knob",
    )
    lid.inertial = Inertial.from_geometry(
        Box((2.0 * r.top_r, 2.0 * r.top_r, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(-hinge_x, 0.0, seat_dz)),
    )
    model.articulation(
        "bucket_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, rim_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=1.5, lower=0.0, upper=math.radians(110)
        ),
        mating=MatingContract(
            parent_face_geometry="hinge_barrel",
            parent_face_side="positive_z",
            child_face_geometry="hinge_strap",
            child_face_side="negative_z",
            contact_tol=0.016,
        ),
    )
    return ["lid"]


_HANDLE_BUILDERS = {
    "swing_bail_revolute": _emit_swing_bail,
    "fixed_side_grips": _emit_side_grips,
    "hinged_lid": _emit_hinged_lid,
}


# ---------------------------------------------------------------------------
# Slot C: mounting.
# ---------------------------------------------------------------------------
def _emit_hook_ring(model, r: ResolvedBucket1Config, body, mats) -> None:
    """Decorative hanging ring above the rim: 4 arms + plate + shank + torus.
    All inline body visuals (Rule 1, no new joint)."""
    rim_z = r.body_h - r.rim_tube * 0.4  # rim torus plane
    # Four horizontal radial arms at the rim plane, each spanning from the center
    # out PAST the rim (outer end embedded in the rolled rim torus), so the hook
    # assembly is fused to the body — no island. The central plate rests on them.
    arm_len = r.top_r + r.rim_tube  # reach into the rim torus
    for i in range(4):
        ang = math.pi / 2.0 * i
        rmid = arm_len * 0.5
        body.visual(
            Box((arm_len, 0.010, 0.010)),
            origin=Origin(
                xyz=(rmid * math.cos(ang), rmid * math.sin(ang), rim_z),
                rpy=(0.0, 0.0, ang),
            ),
            material=mats["wire"],
            name=f"hook_arm_{i}",
        )
    plate_z = rim_z + 0.010
    body.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, plate_z)),
        material=mats["accent"],
        name="hook_plate",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, plate_z + 0.018)),
        material=mats["wire"],
        name="hook_shank",
    )
    ring = TorusGeometry(0.018, 0.005, radial_segments=10, tubular_segments=24)
    body.visual(
        mesh_from_geometry(ring, "hook_ring"),
        origin=Origin(xyz=(0.0, 0.0, plate_z + 0.040), rpy=(0.5 * math.pi, 0.0, 0.0)),
        material=mats["wire"],
        name="hook_ring",
    )


def _emit_wall_bracket(model: ArticulatedObject, r: ResolvedBucket1Config, mats):
    """Wall bracket = root part. Vertical back plate (2 bolt holes) + cradle ring
    (hugging the wall at cradle_z_frac) + arm. Reparents bucket via FIXED."""
    bracket = model.part("bracket")
    cradle_z = r.cradle_z_frac * r.body_h
    wall_r = _radius_at(r, cradle_z)
    # The bucket shell AABB max-X is its widest radius (the mouth, top_r). The
    # cradle arm's inner face reaches that plane so the FIXED mate contacts the
    # shell AABB on the +X side, while the cradle ring visually hugs the wall.
    seat_r = max(wall_r, r.top_r)  # the shell's widest plane on +X
    plate_x = seat_r + 0.030  # back plate sits at +X past the bucket
    # Back plate spanning from the floor plane up the back, so the down-spine
    # and cradle arm both fuse to it (no island).
    plate_h = r.body_h * 0.85
    plate_zc = plate_h * 0.5
    bracket.visual(
        Box((0.012, 0.110, plate_h)),
        origin=Origin(xyz=(plate_x, 0.0, plate_zc)),
        material=mats["lug"],
        name="back_plate",
    )
    for i, sz in ((0, 1.0), (1, -1.0)):
        bracket.visual(
            Cylinder(radius=0.006, length=0.016),
            origin=Origin(xyz=(plate_x + 0.006, 0.0, plate_zc + sz * plate_h * 0.32), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=mats["accent"],
            name=f"bolt_hole_{i}",
        )
    # Cradle arm reaching in from the plate to the shell widest plane (inner face
    # at seat_r), so its negative_x AABB face coincides with the shell positive_x
    # face for a tight FIXED mate; it also bridges to the cradle ring.
    arm_inner = seat_r - 0.004  # just embed into the shell widest plane
    bracket.visual(
        Box((plate_x - arm_inner, 0.024, 0.018)),
        origin=Origin(xyz=(0.5 * (plate_x + arm_inner), 0.0, cradle_z)),
        material=mats["lug"],
        name="cradle_arm",
    )
    # Cradle ring hugging the wall at cradle height (full torus at the local wall
    # radius); its +X side reaches seat_r, fusing to the arm.
    ring = TorusGeometry(
        seat_r - r.band_tube, r.band_tube * 1.3, radial_segments=10, tubular_segments=48
    )
    bracket.visual(
        mesh_from_geometry(ring, "cradle_ring"),
        origin=Origin(xyz=(0.0, 0.0, cradle_z)),
        material=mats["lug"],
        name="cradle_ring",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 0.16)), mass=0.6, origin=Origin(xyz=(plate_x * 0.5, 0.0, r.body_h * 0.5))
    )
    # Bracket base foot under the bucket floor center, so the FIXED joint origin
    # (0,0,0) — where the bucket stays concentric with the bracket frame — lands
    # on real bracket geometry (the foot) AND the bucket floor.
    bracket.visual(
        Box((0.060, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=mats["lug"],
        name="bracket_foot",
    )
    # Down-spine connecting the back plate to the foot (no island).
    bracket.visual(
        Box((plate_x, 0.024, 0.016)),
        origin=Origin(xyz=(plate_x * 0.5, 0.0, 0.006)),
        material=mats["lug"],
        name="bracket_spine",
    )
    # FIXED mate datum: bucket concentric with the bracket frame; the origin at
    # (0,0,0) sits on the bracket foot and the bucket floor.
    mate_origin = (0.0, 0.0, 0.0)
    return bracket, mate_origin


# ---------------------------------------------------------------------------
# slot_choices
# ---------------------------------------------------------------------------
def slot_choices_for_config(
    config: Bucket1Config | ResolvedBucket1Config,
) -> tuple[tuple[str, str], ...]:
    r = config if isinstance(config, ResolvedBucket1Config) else resolve_config(config)
    choices = [
        ("body_profile", r.body_profile),
        ("handle", r.handle),
        ("mount", r.mount),
    ]
    if r.band_count > 0:
        choices.append(("band_count", f"n{r.band_count}"))
    else:
        choices.append(("band_count", "rolled_rim_only"))
    return tuple(choices)


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def _build_body(model: ArticulatedObject, r: ResolvedBucket1Config, mats):
    body = model.part("bucket")
    _emit_body_shell(body, r, mats)
    _emit_rolled_rim(body, r, mats)
    if r.band_count > 0:
        _emit_bands(body, r, mats)
    if r.mount == "hook_ring":
        _emit_hook_ring(model, r, body, mats)
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=max(r.top_r, 0.05), length=r.body_h),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, r.body_h / 2.0)),
    )
    return body


def build_bucket1(
    config: Bucket1Config | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=r.name, assets=assets)
    mats = {
        key: model.material(f"bucket1_{key}_{r.palette_style}", rgba=rgba)
        for key, rgba in PALETTES[r.palette_style].items()
    }

    body = _build_body(model, r, mats)

    # Slot C: wall_bracket introduces a new root (bracket) + FIXED reparent.
    if r.mount == "wall_bracket":
        bracket, mate_origin = _emit_wall_bracket(model, r, mats)
        model.articulation(
            "bracket_to_bucket",
            ArticulationType.FIXED,
            parent=bracket,
            child=body,
            origin=Origin(xyz=mate_origin),
            mating=MatingContract(
                parent_face_geometry="bracket_foot",
                parent_face_side="positive_z",
                child_face_geometry="bucket_shell",
                child_face_side="negative_z",
                contact_tol=0.020,
            ),
        )

    # Slot B: handle (defining joint).
    _HANDLE_BUILDERS[r.handle](model, r, body, mats)

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_bucket1(
    seed: int,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    return build_bucket1(config_from_seed(seed), assets=assets)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_bucket1_tests(
    object_model: ArticulatedObject,
    config: Bucket1Config,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    body = object_model.get_part("bucket")

    # ---- Overlap allowances. ----
    if r.handle == "swing_bail_revolute":
        handle = object_model.get_part("handle")
        ctx.allow_overlap(
            handle, body,
            reason="bail-wire end seats are riveted into the ±Y pivot lugs.",
        )
    elif r.handle == "fixed_side_grips":
        fold = object_model.get_part("fold_grip")
        ctx.allow_overlap(
            fold, body,
            reason="fold-flat grip pivot anchor rides on the +Y grip lug.",
        )
    else:  # hinged_lid
        lid = object_model.get_part("lid")
        ctx.allow_overlap(
            lid, body,
            reason="closed lid seats on the rolled rim and hinge barrel.",
        )

    if r.mount == "wall_bracket":
        bracket = object_model.get_part("bracket")
        ctx.allow_overlap(
            bracket, body,
            reason="cradle ring hugs the bucket wall (FIXED mount cradle).",
        )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_joint_mating_has_gap()

    # ---- Structure / identity checks. ----
    part_names = {p.name for p in object_model.parts}
    ctx.check("bucket body present", "bucket" in part_names, details=str(sorted(part_names)))

    # Body is a surface-of-revolution shell (Rule 3) — named lathe visual.
    wall_names = {v.name for v in body.visuals}
    ctx.check(
        "body has a lathe revolve shell (not Box/Cylinder downgrade)",
        "bucket_shell" in wall_names,
        details=str(sorted(wall_names)),
    )
    # Rolled rim is the identity invariant (present for every variant).
    ctx.check(
        "rolled rim present (sheet-metal fire-bucket identity invariant)",
        "rolled_rim" in wall_names,
        details=str(sorted(wall_names)),
    )

    # Footprint / standing-vs-hanging (cone family hangs, apex lowest).
    aabb = ctx.part_world_aabb(body)
    if aabb is not None:
        height = aabb[1][2] - aabb[0][2]
        ctx.check(
            "bucket has real depth >= 0.18",
            height > 0.18,
            details=f"height={height:.3f}",
        )
        if not r.is_cone:
            ctx.check(
                "flat-bottom bucket rests near z~0",
                abs(aabb[0][2]) < 0.030,
                details=f"min_z={aabb[0][2]:.4f}",
            )

    # Band multiplicity present with correct naming (Rule 1, inline visuals).
    if r.band_count > 0:
        bands = [v.name for v in body.visuals if v.name.startswith("band_")]
        ctx.check(
            "N reinforcing bands inlined (Rule 1)",
            len(bands) == r.band_count,
            details=f"bands={len(bands)} expected={r.band_count}",
        )
        # Bands hug the wall: each at the local wall radius, within the body.
        for z in _band_heights(r):
            ctx.check(
                f"band z={z:.3f} within [0.08,0.92]*body_h",
                r.body_h * 0.08 - 1e-6 <= z <= r.body_h * 0.92 + 1e-6,
                details=f"z={z:.4f} body_h={r.body_h:.3f}",
            )

    # ---- At least one real non-FIXED joint (defining bail/grip/lid). ----
    non_fixed = [
        a for a in object_model.articulations if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least one non-fixed defining joint (bail / grip / lid)",
        len(non_fixed) >= 1,
        details=f"non_fixed={[a.name for a in non_fixed]}",
    )

    # ---- Defining handle joint topology. ----
    if r.handle == "swing_bail_revolute":
        j = object_model.get_articulation("bucket_to_handle")
        ctx.check(
            "bail is REVOLUTE about ±Y diameter line",
            j.articulation_type == ArticulationType.REVOLUTE and abs(j.axis[1]) > 0.99,
            details=f"type={j.articulation_type} axis={tuple(j.axis)}",
        )
        # Joint origin sits on the lug pivot axis (0,0,lug_z).
        ox, oy, oz = j.origin.xyz
        ctx.check(
            "bail joint origin on lug axis (0,0,lug_z)",
            abs(ox) < 1e-6 and abs(oy) < 1e-6 and abs(oz - r.lug_z) < 1e-6,
            details=f"origin=({ox:.4f},{oy:.4f},{oz:.4f}) lug_z={r.lug_z:.4f}",
        )
        # Bail linkage stands outside the rim (clearance inequality).
        ctx.check(
            "bail linkage clears the rim (lug_y - wire_r > rim_outer_y)",
            (r.lug_y - _WIRE_R) > r.rim_outer_y + 0.0005,
            details=f"lug_y={r.lug_y:.4f} rim_outer_y={r.rim_outer_y:.4f}",
        )
        # Actuation: swinging the bail moves the apex sideways.
        handle = object_model.get_part("handle")
        with ctx.pose({j: 0.0}):
            a0 = ctx.part_world_aabb(handle)
        with ctx.pose({j: math.radians(90)}):
            a1 = ctx.part_world_aabb(handle)
        if a0 is not None and a1 is not None:
            ctx.check(
                "bail swings (apex drops when rotated to the side)",
                a1[1][2] < a0[1][2] - 0.02,
                details=f"upright_top={a0[1][2]:.3f} side_top={a1[1][2]:.3f}",
            )
    elif r.handle == "fixed_side_grips":
        j = object_model.get_articulation("bucket_to_fold_grip")
        ctx.check(
            "fold grip is REVOLUTE about ±X",
            j.articulation_type == ArticulationType.REVOLUTE and abs(j.axis[0]) > 0.99,
            details=f"type={j.articulation_type} axis={tuple(j.axis)}",
        )
    else:  # hinged_lid
        j = object_model.get_articulation("bucket_to_lid")
        ctx.check(
            "hinged lid is REVOLUTE about ±Y rim-tangent line",
            j.articulation_type == ArticulationType.REVOLUTE and abs(j.axis[1]) > 0.99,
            details=f"type={j.articulation_type} axis={tuple(j.axis)}",
        )
        # hinged_lid removes lugs/bail (no double-handle semantics).
        ctx.check(
            "hinged_lid removed pivot lugs / bail",
            "lug_pos" not in wall_names and "handle" not in part_names,
            details=str(sorted(wall_names)),
        )
        # Lid opens upward/outward.
        lid = object_model.get_part("lid")
        with ctx.pose({j: 0.0}):
            c = ctx.part_world_aabb(lid)
        with ctx.pose({j: math.radians(100)}):
            o = ctx.part_world_aabb(lid)
        if c is not None and o is not None:
            ctx.check(
                "lid swings open (top rises)",
                o[1][2] > c[1][2] + 0.02,
                details=f"closed_top={c[1][2]:.3f} open_top={o[1][2]:.3f}",
            )

    # ---- Mount topology. ----
    if r.mount == "wall_bracket":
        j = object_model.get_articulation("bracket_to_bucket")
        ctx.check(
            "wall bracket reparents bucket via FIXED",
            j.articulation_type == ArticulationType.FIXED,
            details=f"type={j.articulation_type}",
        )
        bracket = object_model.get_part("bracket")
        bnames = {v.name for v in bracket.visuals}
        ctx.check(
            "bracket has back plate + cradle ring",
            "back_plate" in bnames and "cradle_ring" in bnames,
            details=str(sorted(bnames)),
        )
    elif r.mount == "hook_ring":
        ctx.check(
            "hook ring inline visuals present (decorative, no joint)",
            "hook_ring" in wall_names and "hook_shank" in wall_names,
            details=str(sorted(wall_names)),
        )

    # ---- slot_choices recorded. ----
    ctx.check(
        "slot_choices_recorded",
        tuple(object_model.meta.get("slot_choices", ())) == slot_choices_for_config(r),
        details=str(object_model.meta.get("slot_choices")),
    )

    return ctx.report()


__all__ = (
    "Bucket1Config",
    "ResolvedBucket1Config",
    "build_bucket1",
    "build_seeded_bucket1",
    "config_from_seed",
    "resolve_config",
    "run_bucket1_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
    "with_overrides",
)
