"""Tipping barrow / tilt-truck dump cart — modular procedural template.

A wheeled steel ``frame`` (root) carries a sheet-metal ``tub``/``hopper`` load
body that **tips forward (+X) about a lateral (Y) REVOLUTE pivot** — the
category-defining joint. The ground is met by 1/2/4 wheels rolling on always-on
CONTINUOUS joints, and the cart rests on stand legs / an axle cradle / swivel
casters when stationary.

Two source families (spines) share one abstract topology
``frame(root) → {tub tip REVOLUTE, N×wheel CONTINUOUS roll, support}``:

  * **barrow spine** — tubular handle frame + lofted sheet-metal bowl/pan +
    stand legs + 1 or 2 wheels (S1/S2/S6/S8).
  * **tilt-truck spine** — low box base frame + tapered/deep ribbed PE hopper +
    2 big wheels (+ 2 front swivel casters at N=4) (S3/S4/S5/S7/S9).

Slot graph (pattern = mixed: root parallel children + one multiplicity axis +
an optional ``lift_arm`` chain link):

    frame_body (root, steel)
        → tip_mechanism   (emits the tub/hopper part + the tip REVOLUTE,
                            or a lift_arm→hopper 2-hop chain, or FIXED tub)
        → wheels          (N×wheel CONTINUOUS roll; N=4 ⇒ front swivel casters)
        → support         (stand legs / axle cradle / front swivel casters)

The defining joint is ALWAYS present: ``front_pivot_tip`` / ``lift_and_tip``
give the tub/hopper a tip REVOLUTE (axis ±Y, lower=0); ``lift_off_then_lift``
keeps the tub FIXED on the frame but the wheels' CONTINUOUS roll carries the
required non-fixed DOF (a fully static model is never produced).

Coordinate frame: Z-up, floor z=0, +X = forward / tipping direction,
+Y = lateral axle line, centerline y=0. All mirrored pieces straddle y=0.

Adopted sources (see spec Adopted Source Index):
  S1/S2  single/two-wheel garden barrow — lofted bowl tray, tubular frame
  S3/S5  tilt-truck — tapered/deep ribbed PE hopper, low box frame, big wheels
  S4     four-wheel tilt-truck — rear roll + front swivel casters
  S6     rounded-pan barrow — elliptical lofted pan
  S7     lift-and-tip — U lift_arm two-hop REVOLUTE chain
  S8     barrow with split named-visual legs + foot rail
  S9     swivel-caster assembly (kingpin yaw + wheel roll)
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Literal

from agent.templates._modular import (
    InterfaceSpec,
    ModuleBuild,
    ModuleBuildContext,
    SlotSpec,
    assemble,
)
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)

# Modular template: sweep coverage uses module_topology_diversity.
__modular__ = True


# --------------------------------------------------------------------------- #
# Module enums
# --------------------------------------------------------------------------- #


TubShapeModule = Literal[
    "shallow_tray",      # barrow spine
    "rounded_pan",       # barrow spine
    "tapered_hopper",    # tilt-truck spine
    "deep_dump_tub",     # tilt-truck spine
]
TipMechanismModule = Literal[
    "front_pivot_tip",
    "lift_off_then_lift",
    "lift_and_tip",
]
WheelCountModule = Literal[
    "one_front_wheel",   # N=1 barrow
    "two_axle_wheels",   # N=2
    "four_wheels",       # N=4 tilt-truck (front swivel casters)
]
SupportModule = Literal[
    "stand_legs",            # barrow
    "axle_cradle",           # tilt-truck
    "front_swivel_casters",  # tilt-truck
]
TippingBarrowPalette = Literal[
    "garden_steel",
    "industrial_gray",
    "safety_orange",
    "agri_green",
    "municipal_blue",
    "galv_zinc",
]

_BARROW_TUBS = ("shallow_tray", "rounded_pan")
_TILT_TUBS = ("tapered_hopper", "deep_dump_tub")


# --------------------------------------------------------------------------- #
# Palette presets (palette_style; >=3 colorways, here 6). Each drives the same
# token set so every .visual references a registered material.
# --------------------------------------------------------------------------- #


PALETTES: dict[str, dict[str, tuple[float, float, float, float]]] = {
    "garden_steel": {
        "tub_paint": (0.46, 0.47, 0.49, 1.0),
        "frame_steel": (0.62, 0.63, 0.66, 1.0),
        "rubber": (0.06, 0.06, 0.06, 1.0),
        "rim_paint": (0.30, 0.31, 0.34, 1.0),
        "hub_steel": (0.55, 0.56, 0.52, 1.0),
        "grip": (0.86, 0.78, 0.20, 1.0),
        "accent": (0.78, 0.80, 0.82, 1.0),
    },
    "industrial_gray": {
        "tub_paint": (0.62, 0.63, 0.64, 1.0),
        "frame_steel": (0.34, 0.35, 0.37, 1.0),
        "rubber": (0.07, 0.07, 0.07, 1.0),
        "rim_paint": (0.45, 0.46, 0.48, 1.0),
        "hub_steel": (0.50, 0.51, 0.53, 1.0),
        "grip": (0.14, 0.14, 0.15, 1.0),
        "accent": (0.72, 0.73, 0.75, 1.0),
    },
    "safety_orange": {
        "tub_paint": (0.85, 0.40, 0.10, 1.0),
        "frame_steel": (0.12, 0.13, 0.14, 1.0),
        "rubber": (0.06, 0.06, 0.06, 1.0),
        "rim_paint": (0.20, 0.20, 0.22, 1.0),
        "hub_steel": (0.55, 0.56, 0.58, 1.0),
        "grip": (0.90, 0.78, 0.16, 1.0),
        "accent": (0.92, 0.50, 0.16, 1.0),
    },
    "agri_green": {
        "tub_paint": (0.20, 0.45, 0.22, 1.0),
        "frame_steel": (0.40, 0.42, 0.44, 1.0),
        "rubber": (0.05, 0.05, 0.05, 1.0),
        "rim_paint": (0.16, 0.34, 0.18, 1.0),
        "hub_steel": (0.52, 0.54, 0.50, 1.0),
        "grip": (0.10, 0.10, 0.10, 1.0),
        "accent": (0.80, 0.74, 0.20, 1.0),
    },
    "municipal_blue": {
        "tub_paint": (0.20, 0.35, 0.62, 1.0),
        "frame_steel": (0.40, 0.42, 0.45, 1.0),
        "rubber": (0.08, 0.08, 0.09, 1.0),
        "rim_paint": (0.18, 0.30, 0.54, 1.0),
        "hub_steel": (0.55, 0.56, 0.58, 1.0),
        "grip": (0.84, 0.86, 0.88, 1.0),
        "accent": (0.78, 0.80, 0.82, 1.0),
    },
    "galv_zinc": {
        "tub_paint": (0.70, 0.72, 0.74, 1.0),
        "frame_steel": (0.66, 0.68, 0.70, 1.0),
        "rubber": (0.06, 0.06, 0.06, 1.0),
        "rim_paint": (0.60, 0.62, 0.64, 1.0),
        "hub_steel": (0.74, 0.76, 0.78, 1.0),
        "grip": (0.20, 0.20, 0.22, 1.0),
        "accent": (0.80, 0.82, 0.84, 1.0),
    },
}

PALETTE_NAMES: tuple[str, ...] = tuple(PALETTES.keys())


# --------------------------------------------------------------------------- #
# Config dataclasses
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class TippingBarrowConfig:
    """Public tipping_barrow config. Module fields left ``None`` fall back to
    the spine-consistent anchor in ``resolve_config``."""

    tub_shape_module: TubShapeModule | None = None
    tip_mechanism_module: TipMechanismModule | None = None
    wheel_count_module: WheelCountModule | None = None
    support_module: SupportModule | None = None
    palette_style: TippingBarrowPalette = "garden_steel"

    # Continuous scales (independent unless noted).
    tub_depth_scale: float = 1.0
    tub_len_scale: float = 1.0
    tub_wid_scale: float = 1.0  # derived = tub_len_scale
    wheel_radius_scale: float = 1.0
    track_scale: float = 1.0
    tip_upper: float = 1.1
    handle_len_scale: float = 1.0


@dataclass(frozen=True)
class ResolvedTippingBarrowConfig:
    """Clamped + spine-resolved config consumed by builders."""

    tub_shape_module: TubShapeModule
    tip_mechanism_module: TipMechanismModule
    wheel_count_module: WheelCountModule
    support_module: SupportModule
    palette_style: str
    spine: str  # "barrow" | "tilt_truck"

    tub_depth_scale: float
    tub_len_scale: float
    tub_wid_scale: float
    wheel_radius_scale: float
    track_scale: float
    tip_upper: float
    handle_len_scale: float

    # Derived geometry (canonical frame).
    wheel_radius: float
    wheel_width: float
    hub_radius: float
    axle_x: float       # rear axle X
    axle_z: float       # wheel center height = wheel_radius
    half_track: float   # |y| of each wheel
    pivot_x: float      # tip pivot X (== rear axle line)
    pivot_z: float      # tip pivot height
    tub_len: float
    tub_wid: float
    tub_depth: float
    rim_z: float        # tub rim top (authored world, pre-pivot)
    frame_top_z: float  # top of frame deck (tub rests above)

    palette: dict[str, tuple[float, float, float, float]]

    @property
    def wheel_count(self) -> int:
        return {"one_front_wheel": 1, "two_axle_wheels": 2, "four_wheels": 4}[
            self.wheel_count_module
        ]


# --------------------------------------------------------------------------- #
# Seed-driven sampling (procedural-first; spine-gated)
# --------------------------------------------------------------------------- #


def config_from_seed(seed: int) -> TippingBarrowConfig:
    """Deterministic procedural sampling, spine-first.

    seed 0 is the canonical tilt-truck root (tapered_hopper + front_pivot_tip +
    two big wheels + axle cradle). Other seeds draw tub_shape (which fixes the
    spine), then conditionally draw wheel_count / tip / support / palette per
    the compatibility matrix, then continuous scales.
    """
    if seed == 0:
        return TippingBarrowConfig(
            tub_shape_module="tapered_hopper",
            tip_mechanism_module="front_pivot_tip",
            wheel_count_module="two_axle_wheels",
            support_module="axle_cradle",
            palette_style="industrial_gray",
        )

    rng = random.Random(seed)

    # tub_shape first → determines spine.
    tub_shape = rng.choice(("shallow_tray", "rounded_pan", "tapered_hopper", "deep_dump_tub"))
    spine = "barrow" if tub_shape in _BARROW_TUBS else "tilt_truck"

    if spine == "barrow":
        wheel_count = rng.choice(("one_front_wheel", "two_axle_wheels"))
        tip = rng.choice(("front_pivot_tip", "lift_off_then_lift"))
        support = "stand_legs"
    else:
        # N weighting: N=2 common, N=4 rarer. N=4 ⇒ casters.
        wheel_count = rng.choices(
            ("two_axle_wheels", "four_wheels"), weights=(0.6, 0.4)
        )[0]
        tip = rng.choice(("front_pivot_tip", "lift_and_tip"))
        if wheel_count == "four_wheels":
            support = "front_swivel_casters"
        else:
            support = rng.choice(("axle_cradle", "front_swivel_casters"))

    palette = rng.choice(PALETTE_NAMES)

    tub_depth_scale = round(rng.uniform(0.85, 1.30), 4)
    tub_len_scale = round(rng.uniform(0.90, 1.15), 4)
    wheel_radius_scale = round(rng.uniform(0.90, 1.15), 4)
    track_scale = round(rng.uniform(0.90, 1.15), 4)
    tip_upper = round(rng.uniform(0.7, 1.4), 4)
    handle_len_scale = round(rng.uniform(0.90, 1.15), 4)

    return TippingBarrowConfig(
        tub_shape_module=tub_shape,
        tip_mechanism_module=tip,
        wheel_count_module=wheel_count,
        support_module=support,
        palette_style=palette,
        tub_depth_scale=tub_depth_scale,
        tub_len_scale=tub_len_scale,
        tub_wid_scale=tub_len_scale,
        wheel_radius_scale=wheel_radius_scale,
        track_scale=track_scale,
        tip_upper=tip_upper,
        handle_len_scale=handle_len_scale,
    )


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(float(v), hi))


def resolve_config(config: TippingBarrowConfig) -> ResolvedTippingBarrowConfig:
    """Validate enums, fill spine-consistent anchors, clamp/derive/project."""
    tub_shape = config.tub_shape_module or "tapered_hopper"
    spine = "barrow" if tub_shape in _BARROW_TUBS else "tilt_truck"

    # Defaults + spine gating for the other slots.
    if spine == "barrow":
        tip = config.tip_mechanism_module or "front_pivot_tip"
        if tip not in ("front_pivot_tip", "lift_off_then_lift"):
            tip = "front_pivot_tip"
        wheel = config.wheel_count_module or "two_axle_wheels"
        if wheel not in ("one_front_wheel", "two_axle_wheels"):
            wheel = "two_axle_wheels"
        support = "stand_legs"
    else:
        tip = config.tip_mechanism_module or "front_pivot_tip"
        if tip not in ("front_pivot_tip", "lift_and_tip"):
            tip = "front_pivot_tip"
        wheel = config.wheel_count_module or "two_axle_wheels"
        if wheel not in ("two_axle_wheels", "four_wheels"):
            wheel = "two_axle_wheels"
        support = config.support_module or "axle_cradle"
        if support not in ("axle_cradle", "front_swivel_casters"):
            support = "axle_cradle"
        if wheel == "four_wheels":
            support = "front_swivel_casters"

    if str(config.palette_style) not in PALETTES:
        raise ValueError(f"Unsupported palette_style: {config.palette_style}")

    # --- continuous scales ---
    tub_depth_scale = _clamp(config.tub_depth_scale, 0.85, 1.30)
    tub_len_scale = _clamp(config.tub_len_scale, 0.90, 1.15)
    tub_wid_scale = tub_len_scale  # equation
    wheel_radius_scale = _clamp(config.wheel_radius_scale, 0.90, 1.15)
    track_scale = _clamp(config.track_scale, 0.90, 1.15)
    tip_upper = _clamp(config.tip_upper, 0.7, 1.4)
    handle_len_scale = _clamp(config.handle_len_scale, 0.90, 1.15)

    # --- wheel geometry (spine-specific nominal radius) ---
    base_r = 0.18 if spine == "barrow" else 0.205
    wheel_radius = base_r * wheel_radius_scale
    wheel_width = 0.085 if spine == "barrow" else 0.11
    hub_radius = 0.046
    axle_z = wheel_radius  # ground contact: center height = radius

    # --- track / placement ---
    if wheel == "one_front_wheel":
        half_track = 0.0
        base_track = 0.0
    else:
        base_track = 0.28 if spine == "barrow" else 0.345
        half_track = base_track * track_scale

    # --- tub footprint (spine nominal) ---
    if spine == "barrow":
        nom_len, nom_wid, nom_depth = 0.78, 0.60, 0.27
    else:
        nom_len, nom_wid, nom_depth = 0.86, 0.66, 0.46
    tub_len = nom_len * tub_len_scale
    tub_wid = nom_wid * tub_wid_scale
    tub_depth = nom_depth * tub_depth_scale

    # Body-not-hit-wheel projection (tilt-truck): tub half-width must clear the
    # wheel y. If too wide, pull the wheels outboard (and/or clamp tub_wid).
    if wheel != "one_front_wheel":
        tub_half_y = tub_wid * 0.5
        min_clear = tub_half_y + wheel_width * 0.5 + 0.02
        if half_track < min_clear:
            half_track = min_clear

    # --- frame deck top: tub rests above the wheels ---
    frame_top_z = axle_z + wheel_radius * 0.35 + 0.05

    # The tub body sits with its bottom on the frame deck; rim above.
    rim_z = frame_top_z + tub_depth

    # --- axle position: barrow wheel(s) sit at the FRONT (+x); tilt-truck big
    #     wheels sit at the REAR axle (-x). The tip pivot is always at the rear
    #     of the tub floor so the load dumps forward over the wheel(s). ---
    if spine == "barrow":
        axle_x = 0.5 * tub_len + wheel_radius + 0.06
    else:
        axle_x = -0.5 * tub_len - 0.02
    pivot_x = -0.5 * tub_len + 0.05  # rear edge of the tub floor
    pivot_z = frame_top_z          # at the deck / tub-bottom line

    palette = dict(PALETTES[config.palette_style])

    return ResolvedTippingBarrowConfig(
        tub_shape_module=tub_shape,
        tip_mechanism_module=tip,
        wheel_count_module=wheel,
        support_module=support,
        palette_style=config.palette_style,
        spine=spine,
        tub_depth_scale=tub_depth_scale,
        tub_len_scale=tub_len_scale,
        tub_wid_scale=tub_wid_scale,
        wheel_radius_scale=wheel_radius_scale,
        track_scale=track_scale,
        tip_upper=tip_upper,
        handle_len_scale=handle_len_scale,
        wheel_radius=wheel_radius,
        wheel_width=wheel_width,
        hub_radius=hub_radius,
        axle_x=axle_x,
        axle_z=axle_z,
        half_track=half_track,
        pivot_x=pivot_x,
        pivot_z=pivot_z,
        tub_len=tub_len,
        tub_wid=tub_wid,
        tub_depth=tub_depth,
        rim_z=rim_z,
        frame_top_z=frame_top_z,
        palette=palette,
    )


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #


def _origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 1e-9:
        raise ValueError("zero-length tube")
    nx, ny, nz = vx / length, vy / length, vz / length
    horizontal = math.sqrt(nx * nx + ny * ny)
    pitch = math.atan2(horizontal, nz)
    yaw = math.atan2(ny, nx) if horizontal > 1e-9 else 0.0
    return Origin(xyz=((sx + ex) / 2, (sy + ey) / 2, (sz + ez) / 2), rpy=(0.0, pitch, yaw)), length


def _tube(part, start, end, radius, *, material, name) -> None:
    origin, length = _origin_between(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _ellipse_loop(a: float, b: float, *, segments: int = 28) -> list[tuple[float, float]]:
    """Elliptical 2D profile (rounded-pan sections)."""
    return [
        (a * math.cos(2.0 * math.pi * i / segments), b * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _lofted_tub_mesh(
    r: ResolvedTippingBarrowConfig,
    *,
    z0: float,
    rounded: bool,
    straight_wall: bool,
) -> MeshGeometry:
    """Lofted sheet-metal pan/hopper shell (hollow outer+inner walls + floor).

    Built bottom→top from rounded-rect (or elliptical) loops, with an inner
    wall offset inward to read as a real pan with wall thickness. ``z0`` is the
    tub-bottom in part frame. ``straight_wall`` gives a deep upright bin (deep
    dump tub); otherwise the walls taper out toward the rim.
    """
    L = r.tub_len
    W = r.tub_wid
    D = r.tub_depth
    wall = 0.05

    # Bottom width fraction: straight bins keep a wide floor; tapered hoppers /
    # bowls narrow toward the floor.
    bottom_frac = 0.62 if straight_wall else 0.50

    def loop(length: float, width: float, z: float) -> list[int]:
        out: list[int] = []
        if rounded:
            pts = _ellipse_loop(length * 0.5, width * 0.5, segments=28)
        else:
            pts = rounded_rect_profile(length, width, min(length, width) * 0.16, corner_segments=6)
        for px, py in pts:
            out.append(mesh.add_vertex(px, py, z))
        return out

    mesh = MeshGeometry()
    outer_bottom = loop(L * bottom_frac, W * bottom_frac, z0 + 0.004)
    outer_top = loop(L, W, z0 + D)
    inner_top = loop(L - 2.0 * wall, W - 2.0 * wall, z0 + D - 0.018)
    inner_bottom = loop(
        max(0.10, L * bottom_frac - 2.0 * wall),
        max(0.08, W * bottom_frac - 2.0 * wall),
        z0 + 0.028,
    )

    def ring(a: list[int], b: list[int]) -> None:
        n = len(a)
        for i in range(n):
            j = (i + 1) % n
            mesh.add_face(a[i], a[j], b[j])
            mesh.add_face(a[i], b[j], b[i])

    def cap(loop_ids: list[int], *, reverse: bool) -> None:
        c = mesh.add_vertex(
            sum(mesh.vertices[i][0] for i in loop_ids) / len(loop_ids),
            sum(mesh.vertices[i][1] for i in loop_ids) / len(loop_ids),
            sum(mesh.vertices[i][2] for i in loop_ids) / len(loop_ids),
        )
        for i in range(len(loop_ids)):
            j = (i + 1) % len(loop_ids)
            if reverse:
                mesh.add_face(c, loop_ids[j], loop_ids[i])
            else:
                mesh.add_face(c, loop_ids[i], loop_ids[j])

    ring(outer_bottom, outer_top)   # outer wall
    ring(outer_top, inner_top)      # rim roll-over
    ring(inner_top, inner_bottom)   # inner wall
    cap(outer_bottom, reverse=True)  # outer floor underside
    cap(inner_bottom, reverse=False)  # inner floor top
    return mesh


# --------------------------------------------------------------------------- #
# Tub / hopper part builder (shared across tip mechanisms). The tub is authored
# in WORLD canonical coords then SEATED so its part frame origin sits at the tip
# pivot — so q=0 returns to the authored pose and the joint origin lies in
# geometry (the saddle/cross-tube straddles the pivot).
# --------------------------------------------------------------------------- #


def _build_tub_visuals(part, r: ResolvedTippingBarrowConfig, *, seat: tuple[float, float, float]) -> None:
    """Emit the lofted tub shell + rolled rim + pivot saddle, all shifted by
    ``-seat`` so the part-frame origin coincides with the tip pivot.

    ``seat`` is the pivot point in world coords; subtracting it re-bases the
    body into the tip child frame.
    """
    sx, sy, sz = seat
    rounded = r.tub_shape_module == "rounded_pan"
    straight = r.tub_shape_module == "deep_dump_tub"
    z0 = r.frame_top_z  # tub bottom rests on frame deck (world)

    shell = _lofted_tub_mesh(r, z0=z0, rounded=rounded, straight_wall=straight)
    shell.translate(-sx, -sy, -sz)
    part.visual(mesh_from_geometry(shell, "tub_shell_mesh"), material="tub_paint", name="tub_shell")

    # Rolled rim ring at the top edge (4 tubes), welded onto the shell top.
    L = r.tub_len * 0.5
    W = r.tub_wid * 0.5
    rim_z = z0 + r.tub_depth - 0.006
    rim_pts = [
        ((-L, W * 0.95, rim_z), (L, W * 0.95, rim_z), "rim_left"),
        ((-L, -W * 0.95, rim_z), (L, -W * 0.95, rim_z), "rim_right"),
        ((L * 0.95, -W * 0.9, rim_z), (L * 0.95, W * 0.9, rim_z), "rim_front"),
        ((-L * 0.95, -W * 0.9, rim_z), (-L * 0.95, W * 0.9, rim_z), "rim_rear"),
    ]
    for (a, b, nm) in rim_pts:
        _tube(
            part,
            (a[0] - sx, a[1] - sy, a[2] - sz),
            (b[0] - sx, b[1] - sy, b[2] - sz),
            0.015,
            material="rim_paint",
            name=nm,
        )

    # (Hopper ribs are emitted separately by _emit_hopper_ribs with the same
    # world→child seat shift.)

    # Pivot saddle / trunnion: a lateral cross tube AT the pivot line (child
    # origin z=0), straddling the joint origin (0,0,0). It sits right under the
    # tub floor (the shell outer floor is ~z0 ≈ pivot_z) so the saddle radius
    # overlaps the shell — the tub is one connected weldment. A short stay runs
    # forward along the floor underside to tie the saddle into the shell body.
    saddle_y = max(0.06, W * 0.55)
    _tube(
        part,
        (0.0, -saddle_y, 0.0),
        (0.0, saddle_y, 0.0),
        0.024,
        material="hub_steel",
        name="pivot_saddle",
    )
    # Stay along the floor underside, from the saddle forward to mid-floor,
    # overlapping both the saddle and the shell (connectivity bridge).
    _tube(
        part,
        (0.0, 0.0, 0.0),
        (r.tub_len * 0.3, 0.0, (z0 + 0.01) - sz),
        0.016,
        material="hub_steel",
        name="saddle_stay",
    )

    part.inertial = Inertial.from_geometry(
        Box((r.tub_len, r.tub_wid, r.tub_depth + 0.1)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, (z0 + r.tub_depth * 0.5) - sz)),
    )


# Re-implement rib seating cleanly (the loop above intentionally leaves ribs to
# this helper to keep the world→child shift consistent).
def _emit_hopper_ribs(part, r: ResolvedTippingBarrowConfig, *, seat: tuple[float, float, float]) -> None:
    if r.spine != "tilt_truck":
        return
    sx, sy, sz = seat
    W = r.tub_wid * 0.5
    z0 = r.frame_top_z
    for k, fx in enumerate((-0.25, 0.0, 0.25)):
        x = fx * r.tub_len
        _tube(
            part,
            (x - sx, -W * 0.9 - sy, z0 + 0.03 - sz),
            (x - sx, W * 0.9 - sy, z0 + 0.03 - sz),
            0.012,
            material="frame_steel",
            name=f"rib_{k}",
        )


# --------------------------------------------------------------------------- #
# Frame (root) builder
# --------------------------------------------------------------------------- #


def _frame_downstream(r: ResolvedTippingBarrowConfig) -> InterfaceSpec:
    """Frame downstream interface at the tip pivot (consumed by tip_mechanism).
    The tip joint type/axis come from the tip module's upstream interface, so
    this just exposes a +z face at the pivot bracket top."""
    return InterfaceSpec(
        interface_name="downstream",
        part_name="frame",
        visual_name="pivot_bracket",
        face_side="positive_z",
        anchor_local=(r.pivot_x, 0.0, r.pivot_z),
        contact_tol=0.004,
    )


def _build_frame(ctx: ModuleBuildContext) -> ModuleBuild:
    """Root steel frame. Barrow spine = tubular handle frame; tilt-truck = low
    box base frame. Both expose a pivot bracket at the tip pivot line and a
    deck the tub rests above, plus the axle/mount datums for wheels + support.
    """
    model = ctx.model
    r: ResolvedTippingBarrowConfig = ctx.config  # type: ignore[assignment]
    frame = model.part("frame")

    frame.inertial = Inertial.from_geometry(
        Box((r.tub_len + 0.6, max(0.4, 2 * r.half_track + 0.2), r.frame_top_z + 0.2)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, r.frame_top_z * 0.45)),
    )

    ax, az = r.axle_x, r.axle_z
    L = r.tub_len * 0.5
    deck_z = r.frame_top_z
    ht = max(r.half_track, 0.10)

    # --- longitudinal base rails spanning the whole frame: from the axle line
    #     to the opposite tub end (deck edge). On the tilt-truck spine the rails
    #     reach forward to the caster mount line so the casters land on them. ---
    caster_front_x = r.tub_len * 0.5 + 0.08
    if ax > 0:
        deck_far_x = -L * 0.95  # barrow: axle front, deck extends rear
    else:
        deck_far_x = caster_front_x  # tilt-truck: rails reach the front casters
    for sgn, sfx in ((1.0, "0"), (-1.0, "1")):
        ry = ht * 0.85
        _tube(
            frame,
            (ax, sgn * ry, az + 0.02),
            (deck_far_x, sgn * ry, deck_z - 0.02),
            0.018,
            material="frame_steel",
            name=f"base_rail_{sfx}",
        )
    # Axle cross tube at the axle line (carries the wheels).
    _tube(
        frame,
        (ax, -ht * 0.95, az + 0.02),
        (ax, ht * 0.95, az + 0.02),
        0.018,
        material="frame_steel",
        name="axle_cross",
    )
    # Axle spindle through the wheel hubs (so wheel joint origin lies in frame).
    _tube(
        frame,
        (ax, -(ht + 0.03), az),
        (ax, ht + 0.03, az),
        0.016,
        material="hub_steel",
        name="axle_spindle",
    )

    # --- deck the tub rests above (real anchoring visual under the saddle).
    #     Span the full base-rail track so the deck overlaps both rails
    #     (no disconnected island). ---
    deck_w = max(2.0 * ht * 0.85 + 0.05, r.tub_wid * 0.85)
    frame.visual(
        Box((r.tub_len * 0.95, deck_w, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, deck_z - 0.015)),
        material="frame_steel",
        name="deck",
    )

    # --- pivot bracket at the tip pivot line (real hinge hardware). Its TOP
    #     face is at exactly r.pivot_z (box centered just below) so the tub
    #     saddle (whose joint origin sits at pivot_z) mates flush. ---
    frame.visual(
        Box((0.06, 2.0 * ht + 0.06, 0.05)),
        origin=Origin(xyz=(r.pivot_x, 0.0, r.pivot_z - 0.025)),
        material="hub_steel",
        name="pivot_bracket",
    )
    # (The pivot bracket sits at the deck line and overlaps the deck box, so it
    # is connected without a separate riser.)

    if r.spine == "barrow":
        # Tubular handles sweeping back from the deck to rear grips. They start
        # at the deck top edge (embedded into the deck) so they're connected.
        grip_x = -L - 0.55 * r.handle_len_scale
        for sgn, sfx in ((1.0, "0"), (-1.0, "1")):
            hy = ht * 0.85
            _tube(
                frame,
                (L * 0.55, sgn * hy, deck_z - 0.005),
                (grip_x, sgn * hy, deck_z + 0.10),
                0.018,
                material="frame_steel",
                name=f"handle_{sfx}",
            )
            _tube(
                frame,
                (grip_x, sgn * hy, deck_z + 0.10),
                (grip_x - 0.14, sgn * hy, deck_z + 0.11),
                0.026,
                material="grip",
                name=f"grip_{sfx}",
            )
    else:
        # Front cross tube tying the base rails at the far deck edge.
        _tube(
            frame,
            (deck_far_x, -ht * 0.85, deck_z - 0.02),
            (deck_far_x, ht * 0.85, deck_z - 0.02),
            0.018,
            material="frame_steel",
            name="front_cross",
        )

    return ModuleBuild(
        module_name="frame",
        parts_emitted=["frame"],
        internal_articulations=[],
        interfaces={"downstream": _frame_downstream(r)},
    )


# --------------------------------------------------------------------------- #
# Slot C — tip_mechanism factories. Each owns the defining joint.
# --------------------------------------------------------------------------- #


def _passthrough_downstream(ctx: ModuleBuildContext) -> InterfaceSpec:
    """Re-export the frame downstream so later parallel-children slots find it.
    """
    r: ResolvedTippingBarrowConfig = ctx.config  # type: ignore[assignment]
    return _frame_downstream(r)


def _build_front_pivot_tip(ctx: ModuleBuildContext) -> ModuleBuild:
    """Tub/hopper is an independent part tipping forward about a single Y
    REVOLUTE at the rear floor line (the defining joint). The joint is emitted
    manually (parent=frame) and grandfathered (captured saddle-over-bracket
    pin), so no MatingContract is declared — the saddle straddles the frame
    pivot bracket (overlap allowed in run_tipping_barrow_tests)."""
    model = ctx.model
    r: ResolvedTippingBarrowConfig = ctx.config  # type: ignore[assignment]
    frame = model.get_part("frame")

    tub = model.part("tub")
    seat = (r.pivot_x, 0.0, r.pivot_z)
    _build_tub_visuals(tub, r, seat=seat)
    _emit_hopper_ribs(tub, r, seat=seat)

    model.articulation(
        "frame_to_tub",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=tub,
        origin=Origin(xyz=(r.pivot_x, 0.0, r.pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=0.0, upper=r.tip_upper),
    )
    return ModuleBuild(
        module_name="front_pivot_tip",
        parts_emitted=["tub"],
        internal_articulations=["frame_to_tub"],
        interfaces={"downstream": _frame_downstream(r)},
    )


def _build_lift_off_then_lift(ctx: ModuleBuildContext) -> ModuleBuild:
    """Barrow baseline — tub FIXED on the frame; the required non-fixed DOF is
    carried by the wheels' CONTINUOUS roll (a static-only model is never made).
    The tub is a separate part FIXED to the frame at the saddle line."""
    model = ctx.model
    r: ResolvedTippingBarrowConfig = ctx.config  # type: ignore[assignment]
    frame = model.get_part("frame")

    tub = model.part("tub")
    seat = (r.pivot_x, 0.0, r.pivot_z)
    _build_tub_visuals(tub, r, seat=seat)
    _emit_hopper_ribs(tub, r, seat=seat)

    model.articulation(
        "frame_to_tub",
        ArticulationType.FIXED,
        parent=frame,
        child=tub,
        origin=Origin(xyz=(r.pivot_x, 0.0, r.pivot_z)),
    )
    return ModuleBuild(
        module_name="lift_off_then_lift",
        parts_emitted=["tub"],
        internal_articulations=["frame_to_tub"],
        interfaces={"downstream": _frame_downstream(r)},
    )


def _build_lift_and_tip(ctx: ModuleBuildContext) -> ModuleBuild:
    """Tilt-truck — insert a U-shaped ``lift_arm`` link: frame --REVOLUTE(-Y)-->
    lift_arm (raise) --REVOLUTE(+Y)--> hopper (tip). Two-hop chain. The lift_arm
    chains onto the frame pivot bracket; the hopper chains onto the lift_arm's
    tub-end pin (both emitted here, parallel-children style)."""
    model = ctx.model
    r: ResolvedTippingBarrowConfig = ctx.config  # type: ignore[assignment]
    frame = model.get_part("frame")

    # lift_arm: pivots on the frame bracket; its tub-end pin sits forward+up.
    arm_len = max(0.20, r.tub_len * 0.30)
    arm_rise = max(0.12, r.tub_depth * 0.45)
    # Keep the arm INBOARD of the big rear wheels (which sit at ±half_track) so
    # the rear eye / side arms don't collide with the tires.
    ht = max(0.08, r.half_track - r.wheel_width * 0.5 - 0.03)
    lift_arm = model.part("lift_arm")
    lift_arm.inertial = Inertial.from_geometry(
        Box((arm_len + 0.06, 2 * ht, arm_rise + 0.06)),
        mass=2.4,
        origin=Origin(xyz=(arm_len * 0.5, 0.0, arm_rise * 0.5)),
    )
    # Rear eye sleeve centered on the part origin (joint origin in geometry).
    lift_arm.visual(
        Cylinder(radius=0.026, length=2 * ht + 0.04),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hub_steel",
        name="arm_rear_eye",
    )
    # Two side arms going forward+up to the tub-end pin.
    pin_local = (arm_len, 0.0, arm_rise)
    for sgn, sfx in ((1.0, "0"), (-1.0, "1")):
        _tube(
            lift_arm,
            (0.0, sgn * ht, 0.0),
            (pin_local[0], sgn * ht, pin_local[2]),
            0.018,
            material="frame_steel",
            name=f"arm_side_{sfx}",
        )
    # Tub-end cross pin (mating face for the hopper).
    _tube(
        lift_arm,
        (pin_local[0], -ht - 0.02, pin_local[2]),
        (pin_local[0], ht + 0.02, pin_local[2]),
        0.022,
        material="hub_steel",
        name="arm_tub_pin",
    )

    # frame → lift_arm REVOLUTE (raise, axis -Y).
    model.articulation(
        "frame_to_lift_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lift_arm,
        origin=Origin(xyz=(r.pivot_x, 0.0, r.pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.0, lower=0.0, upper=1.0),
    )

    # hopper seated at the lift_arm tub-end pin WORLD position.
    pin_world = (r.pivot_x + pin_local[0], 0.0, r.pivot_z + pin_local[2])
    hopper = model.part("tub")
    _build_tub_visuals(hopper, r, seat=pin_world)
    _emit_hopper_ribs(hopper, r, seat=pin_world)

    # lift_arm → hopper REVOLUTE (tip, axis +Y). Joint origin is the pin in the
    # lift_arm's LOCAL frame.
    model.articulation(
        "lift_arm_to_hopper",
        ArticulationType.REVOLUTE,
        parent=lift_arm,
        child=hopper,
        origin=Origin(xyz=pin_local),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0, velocity=1.0, lower=0.0, upper=r.tip_upper
        ),
    )

    return ModuleBuild(
        module_name="lift_and_tip",
        parts_emitted=["lift_arm", "tub"],
        internal_articulations=["frame_to_lift_arm", "lift_arm_to_hopper"],
        interfaces={"downstream": _frame_downstream(r)},
    )


# --------------------------------------------------------------------------- #
# Slot A — wheels (multiplicity axis). Each wheel CONTINUOUS roll, axis Y.
# Front swivel casters add a kingpin yaw REVOLUTE.
# --------------------------------------------------------------------------- #


def _build_wheel_visuals(part, r: ResolvedTippingBarrowConfig) -> None:
    """SDK tire + rim, hub centered on (0,0,0), spin axis along part-frame y
    (rotate_z(pi/2) maps WheelGeometry's local-X spin to local Y)."""
    R = r.wheel_radius
    width = r.wheel_width
    hub_r = r.hub_radius
    tire = mesh_from_geometry(
        TireGeometry(
            R,
            width,
            inner_radius=R * 0.70,
            tread=TireTread(style="block", depth=0.010, count=20, land_ratio=0.55),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        "tire_mesh",
    )
    rim = mesh_from_geometry(
        WheelGeometry(
            R * 0.70,
            width * 0.70,
            rim=WheelRim(inner_radius=R * 0.44, flange_height=0.012, flange_thickness=0.006,
                         bead_seat_depth=0.005),
            hub=WheelHub(
                radius=hub_r,
                width=width * 0.80,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=hub_r * 1.2, hole_diameter=0.006),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.007, window_radius=0.014),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "rim_mesh",
    )
    part.visual(tire, origin=Origin(rpy=(0.0, 0.0, math.pi / 2)), material="rubber", name="tire")
    part.visual(rim, origin=Origin(rpy=(0.0, 0.0, math.pi / 2)), material="rim_paint", name="rim")
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=R, length=width),
        mass=3.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )


def _emit_rolling_wheel(model, frame, r, *, name: str, x: float, y: float) -> str:
    """A wheel part chained directly to the frame via CONTINUOUS roll."""
    wheel = model.part(name)
    _build_wheel_visuals(wheel, r)
    joint = f"frame_to_{name}"
    model.articulation(
        joint,
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(x, y, r.axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=30.0),
    )
    return joint


def _build_one_front_wheel(ctx: ModuleBuildContext) -> ModuleBuild:
    model = ctx.model
    r: ResolvedTippingBarrowConfig = ctx.config  # type: ignore[assignment]
    frame = model.get_part("frame")
    joint = _emit_rolling_wheel(model, frame, r, name="wheel", x=r.axle_x, y=0.0)
    return ModuleBuild(
        module_name="one_front_wheel",
        parts_emitted=["wheel"],
        internal_articulations=[joint],
        interfaces={"downstream": _passthrough_downstream(ctx)},
    )


def _build_two_axle_wheels(ctx: ModuleBuildContext) -> ModuleBuild:
    model = ctx.model
    r: ResolvedTippingBarrowConfig = ctx.config  # type: ignore[assignment]
    frame = model.get_part("frame")
    joints = []
    for i, sgn in enumerate((1.0, -1.0)):
        joints.append(
            _emit_rolling_wheel(
                model, frame, r, name=f"wheel_{i}", x=r.axle_x, y=sgn * r.half_track
            )
        )
    return ModuleBuild(
        module_name="two_axle_wheels",
        parts_emitted=["wheel_0", "wheel_1"],
        internal_articulations=joints,
        interfaces={"downstream": _passthrough_downstream(ctx)},
    )


def _build_caster(model, frame, r, *, i: int, sgn: float, x: float) -> tuple[list[str], list[str]]:
    """Front swivel caster: kingpin yaw REVOLUTE (axis Z) + wheel roll
    CONTINUOUS (axis Y). Returns (parts, joints)."""
    y = sgn * r.half_track
    caster_r = r.wheel_radius * 0.62
    mount_z = caster_r + 0.10  # yoke top mount height on the frame

    # Frame mount plate (real anchoring visual for the kingpin boss) + a post
    # tying it down to the frame deck so it's not a disconnected island.
    frame.visual(
        Box((0.06, 0.06, 0.02)),
        origin=Origin(xyz=(x, y, mount_z)),
        material="frame_steel",
        name=f"caster_mount_{i}",
    )
    # Tie the mount down to the base rail (which reaches this x on the tilt-truck
    # spine) with a VERTICAL post at the mount x, so it clears the trailing
    # caster wheel (which sits behind the mount in -x).
    _tube(
        frame,
        (x, y, mount_z),
        (x, sgn * r.half_track * 0.85, r.frame_top_z - 0.02),
        0.016,
        material="frame_steel",
        name=f"caster_post_{i}",
    )

    yoke = model.part(f"caster_yoke_{i}")
    yoke.inertial = Inertial.from_geometry(
        Box((0.10, 0.10, mount_z)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -mount_z * 0.45)),
    )
    # Kingpin boss centered on the part origin (joint origin in geometry).
    yoke.visual(
        Cylinder(radius=0.022, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="hub_steel",
        name="kingpin_boss",
    )
    # Crown plate tying the kingpin boss to the yoke cheek tops (so the boss is
    # connected to the rest of the yoke).
    yoke.visual(
        Box((0.05, 2.0 * (r.wheel_width * 0.5 + 0.015) + 0.03, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material="frame_steel",
        name="yoke_crown",
    )
    # Yoke cheeks down to the wheel axle (trailing offset well behind the mount
    # so the trailing wheel clears the vertical mount post).
    trail = caster_r * 1.25
    axle_local_z = -mount_z + caster_r
    for cs, csx in ((1.0, "0"), (-1.0, "1")):
        _tube(
            yoke,
            (0.0, cs * (r.wheel_width * 0.5 + 0.015), -0.01),
            (-trail, cs * (r.wheel_width * 0.5 + 0.015), axle_local_z),
            0.012,
            material="frame_steel",
            name=f"yoke_cheek_{csx}",
        )
    # Axle stub through the wheel hub (captured pin) so the wheel touches the
    # yoke — spans the full caster wheel width at the axle center.
    _tube(
        yoke,
        (-trail, -(r.wheel_width * 0.5 + 0.02), axle_local_z),
        (-trail, (r.wheel_width * 0.5 + 0.02), axle_local_z),
        0.013,
        material="hub_steel",
        name="caster_axle_stub",
    )

    model.articulation(
        f"frame_to_caster_yoke_{i}",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=yoke,
        origin=Origin(xyz=(x, y, mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=5.0, lower=-math.pi, upper=math.pi),
    )

    cwheel = model.part(f"caster_wheel_{i}")
    R = caster_r
    cwheel.visual(
        Cylinder(radius=R, length=r.wheel_width * 0.8),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="rubber",
        name="tire",
    )
    cwheel.visual(
        Cylinder(radius=R * 0.5, length=r.wheel_width * 0.9),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hub_steel",
        name="hub",
    )
    cwheel.inertial = Inertial.from_geometry(
        Cylinder(radius=R, length=r.wheel_width),
        mass=1.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        f"caster_yoke_{i}_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=yoke,
        child=cwheel,
        origin=Origin(xyz=(-trail, 0.0, axle_local_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=30.0),
    )
    return (
        [f"caster_yoke_{i}", f"caster_wheel_{i}"],
        [f"frame_to_caster_yoke_{i}", f"caster_yoke_{i}_to_wheel"],
    )


def _build_four_wheels(ctx: ModuleBuildContext) -> ModuleBuild:
    """Rear pair roll directly; front pair are swivel casters."""
    model = ctx.model
    r: ResolvedTippingBarrowConfig = ctx.config  # type: ignore[assignment]
    frame = model.get_part("frame")
    parts: list[str] = []
    joints: list[str] = []
    # Rear roll wheels.
    for i, sgn in enumerate((1.0, -1.0)):
        joints.append(
            _emit_rolling_wheel(
                model, frame, r, name=f"wheel_{i}", x=r.axle_x, y=sgn * r.half_track
            )
        )
        parts.append(f"wheel_{i}")
    # Front swivel casters.
    front_x = r.tub_len * 0.5 + 0.08
    for i, sgn in ((2, 1.0), (3, -1.0)):
        cparts, cjoints = _build_caster(model, frame, r, i=i, sgn=sgn, x=front_x)
        parts.extend(cparts)
        joints.extend(cjoints)
    return ModuleBuild(
        module_name="four_wheels",
        parts_emitted=parts,
        internal_articulations=joints,
        interfaces={"downstream": _passthrough_downstream(ctx)},
    )


# --------------------------------------------------------------------------- #
# Slot D — support factories (parallel children of frame). Casters here reuse
# the same _build_caster helper as the wheels slot for N=2 + casters.
# --------------------------------------------------------------------------- #


def _build_stand_legs(ctx: ModuleBuildContext) -> ModuleBuild:
    """Two mirrored stand legs + a foot rail, fused onto the frame as named
    visuals (no new joint; static support)."""
    model = ctx.model
    r: ResolvedTippingBarrowConfig = ctx.config  # type: ignore[assignment]
    frame = model.get_part("frame")

    L = r.tub_len * 0.5
    ht = max(r.half_track, 0.12)
    leg_x = -L * 0.2
    top_z = r.frame_top_z
    for sgn, sfx in ((1.0, "0"), (-1.0, "1")):
        ly = ht * 0.85
        _tube(
            frame,
            (leg_x, sgn * ly, top_z - 0.02),
            (leg_x - 0.05, sgn * ly, 0.02),
            0.018,
            material="frame_steel",
            name=f"stand_leg_{sfx}",
        )
        frame.visual(
            Box((0.10, 0.06, 0.02)),
            origin=Origin(xyz=(leg_x - 0.05, sgn * ly, 0.012)),
            material="rubber",
            name=f"leg_foot_{sfx}",
        )
    # Foot rail across the centerline tying the two feet.
    _tube(
        frame,
        (leg_x - 0.05, -ht * 0.95, 0.03),
        (leg_x - 0.05, ht * 0.95, 0.03),
        0.014,
        material="frame_steel",
        name="foot_rail",
    )
    return ModuleBuild(
        module_name="stand_legs",
        parts_emitted=[],
        internal_articulations=[],
        interfaces={"downstream": _passthrough_downstream(ctx)},
    )


def _build_axle_cradle(ctx: ModuleBuildContext) -> ModuleBuild:
    """No landing legs — the load rests on the rear big wheels; add the cradle
    blocks tying the rear cross tube (named frame visuals only)."""
    model = ctx.model
    r: ResolvedTippingBarrowConfig = ctx.config  # type: ignore[assignment]
    frame = model.get_part("frame")
    ax, az = r.axle_x, r.axle_z
    ht = max(r.half_track, 0.12)
    for sgn, sfx in ((1.0, "0"), (-1.0, "1")):
        frame.visual(
            Box((0.08, 0.05, 0.06)),
            origin=Origin(xyz=(ax, sgn * ht * 0.6, az + 0.04)),
            material="frame_steel",
            name=f"cradle_block_{sfx}",
        )
    return ModuleBuild(
        module_name="axle_cradle",
        parts_emitted=[],
        internal_articulations=[],
        interfaces={"downstream": _passthrough_downstream(ctx)},
    )


def _build_front_swivel_casters(ctx: ModuleBuildContext) -> ModuleBuild:
    """Two front swivel casters mounted on the frame (kingpin yaw + wheel roll).
    Used by the tilt-truck spine when wheels==N2 to give a 4-point stance.
    Skipped (no-op) when the wheels slot already emitted N=4 casters."""
    model = ctx.model
    r: ResolvedTippingBarrowConfig = ctx.config  # type: ignore[assignment]
    frame = model.get_part("frame")

    # If four_wheels already placed casters (parts caster_yoke_2/3), don't add.
    existing = {p.name for p in model.parts}
    if "caster_yoke_2" in existing:
        return ModuleBuild(
            module_name="front_swivel_casters",
            parts_emitted=[],
            internal_articulations=[],
            interfaces={"downstream": _passthrough_downstream(ctx)},
        )

    front_x = r.tub_len * 0.5 + 0.08
    parts: list[str] = []
    joints: list[str] = []
    for i, sgn in ((2, 1.0), (3, -1.0)):
        cparts, cjoints = _build_caster(model, frame, r, i=i, sgn=sgn, x=front_x)
        parts.extend(cparts)
        joints.extend(cjoints)
    return ModuleBuild(
        module_name="front_swivel_casters",
        parts_emitted=parts,
        internal_articulations=joints,
        interfaces={"downstream": _passthrough_downstream(ctx)},
    )


# --------------------------------------------------------------------------- #
# Slot graph + entry points
# --------------------------------------------------------------------------- #


TIP_FACTORIES = {
    "front_pivot_tip": _build_front_pivot_tip,
    "lift_off_then_lift": _build_lift_off_then_lift,
    "lift_and_tip": _build_lift_and_tip,
}
WHEEL_FACTORIES = {
    "one_front_wheel": _build_one_front_wheel,
    "two_axle_wheels": _build_two_axle_wheels,
    "four_wheels": _build_four_wheels,
}
SUPPORT_FACTORIES = {
    "stand_legs": _build_stand_legs,
    "axle_cradle": _build_axle_cradle,
    "front_swivel_casters": _build_front_swivel_casters,
}


def _slots_for_config(r: ResolvedTippingBarrowConfig) -> list[SlotSpec]:
    return [
        SlotSpec(
            slot_name="frame_body",
            candidates={"frame": _build_frame},
            anchor_choice="frame",
        ),
        SlotSpec(
            slot_name="tip_mechanism",
            candidates={r.tip_mechanism_module: TIP_FACTORIES[r.tip_mechanism_module]},
            anchor_choice=r.tip_mechanism_module,
        ),
        SlotSpec(
            slot_name="wheels",
            candidates={r.wheel_count_module: WHEEL_FACTORIES[r.wheel_count_module]},
            anchor_choice=r.wheel_count_module,
        ),
        SlotSpec(
            slot_name="support",
            candidates={r.support_module: SUPPORT_FACTORIES[r.support_module]},
            anchor_choice=r.support_module,
        ),
    ]


def build_tipping_barrow(
    config: TippingBarrowConfig,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name="tipping_barrow", assets=assets)
    for material_name, rgba in r.palette.items():
        model.material(material_name, rgba=rgba)

    rng = random.Random(0)
    assemble(
        model,
        slots=_slots_for_config(r),
        rng=rng,
        palette=r.palette,
        config=r,
        seed=0,
        selection_mode="anchor_choices",
    )
    return model


def build_seeded_tipping_barrow(seed: int) -> ArticulatedObject:
    return build_tipping_barrow(config_from_seed(seed))


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    cfg = config_from_seed(seed)
    r = resolve_config(cfg)
    return [
        ("tub_shape", r.tub_shape_module),
        ("tip_mechanism", r.tip_mechanism_module),
        ("wheels", r.wheel_count_module),
        ("support", r.support_module),
    ]


# --------------------------------------------------------------------------- #
# Author-layer QC
# --------------------------------------------------------------------------- #


def _declare_overlaps(ctx: TestContext, model: ArticulatedObject) -> None:
    parts = {p.name for p in model.parts}

    def _ov(a: str, b: str, *, reason: str) -> None:
        if a in parts and b in parts:
            ctx.allow_overlap(model.get_part(a), model.get_part(b), reason=reason)

    # tub/hopper pivot saddle straddles the frame pivot bracket / axle.
    _ov("frame", "tub", reason="tub pivot saddle straddles the frame pivot bracket at the axle line")
    _ov("frame", "lift_arm", reason="lift_arm rear eye captured on the frame pivot bracket")
    _ov("lift_arm", "tub", reason="hopper cradle captured on the lift_arm tub-end pin")
    # wheels: hub captured on the axle spindle / frame.
    for nm in ("wheel", "wheel_0", "wheel_1"):
        _ov("frame", nm, reason="wheel hub captured on the frame axle spindle")
    # casters: kingpin boss captured on the frame mount plate.
    for i in (2, 3):
        _ov("frame", f"caster_yoke_{i}", reason="caster kingpin boss captured on the frame mount plate")
        _ov(f"caster_yoke_{i}", f"caster_wheel_{i}", reason="caster wheel captured in the yoke cheeks")


def _declare_islands(ctx: TestContext, model: ArticulatedObject, r: ResolvedTippingBarrowConfig) -> None:
    parts = {p.name for p in model.parts}
    fn = getattr(ctx, "allow_disconnected_islands", None)
    if fn is None:
        return
    if "tub" in parts:
        fn(model.get_part("tub"),
           reason="lofted tub shell + pivot saddle/rim are separated rigid weldments")
    if "lift_arm" in parts:
        fn(model.get_part("lift_arm"),
           reason="lift_arm rear eye + side arms + tub pin are separated rigid weldments")
    for i in (2, 3):
        if f"caster_yoke_{i}" in parts:
            fn(model.get_part(f"caster_yoke_{i}"),
               reason="caster yoke boss + cheeks are separated rigid pieces")


def _find_tip_joint(model: ArticulatedObject):
    """The defining tip REVOLUTE: child=tub, or lift_arm_to_hopper."""
    for j in getattr(model, "articulations", []):
        nm = getattr(j, "name", "")
        if nm in ("frame_to_tub", "lift_arm_to_hopper"):
            if j.articulation_type == ArticulationType.REVOLUTE:
                return j
    return None


def _expect_defining_joint(ctx: TestContext, model: ArticulatedObject, r: ResolvedTippingBarrowConfig) -> None:
    """Defining joint: front_pivot_tip / lift_and_tip have a tip REVOLUTE axis
    ±Y lower=0; lift_off keeps tub FIXED but wheels carry CONTINUOUS roll."""
    if r.tip_mechanism_module in ("front_pivot_tip", "lift_and_tip"):
        tip = _find_tip_joint(model)
        ok = tip is not None
        axis_ok = ok and abs(tuple(tip.axis)[1]) > 0.9
        lim = tip.motion_limits if ok else None
        lower_ok = ok and lim is not None and (lim.lower is None or abs(lim.lower) < 1e-6)
        ctx.check(
            "tip_revolute_present_lateral_lower0",
            bool(ok and axis_ok and lower_ok),
            details=f"tip={getattr(tip,'name',None)} axis={getattr(tip,'axis',None)} lim={lim}",
        )
    # Always at least one CONTINUOUS roll joint.
    conts = [
        j for j in getattr(model, "articulations", [])
        if j.articulation_type == ArticulationType.CONTINUOUS
    ]
    ctx.check(
        "has_rolling_wheel_joint",
        len(conts) >= 1,
        details=f"continuous joints={[j.name for j in conts]}",
    )


def _expect_wheel_count(ctx: TestContext, model: ArticulatedObject, r: ResolvedTippingBarrowConfig) -> None:
    # Primary ground wheels are parts named wheel / wheel_i (casters are
    # caster_wheel_i and counted separately).
    primary = [
        p.name for p in model.parts
        if p.name == "wheel" or (p.name.startswith("wheel_") and "caster" not in p.name)
    ]
    expected_primary = 1 if r.wheel_count == 1 else (2 if r.wheel_count == 2 else 2)
    ctx.check(
        "wheel_count_matches",
        len(primary) == expected_primary,
        details=f"primary wheels={primary} expected={expected_primary} (N={r.wheel_count})",
    )


def _expect_wheels_grounded(ctx: TestContext, model: ArticulatedObject, r: ResolvedTippingBarrowConfig) -> None:
    names = [p.name for p in model.parts if p.name.startswith("wheel")]
    if not names:
        return
    ok = True
    for nm in names:
        aabb = ctx.part_world_aabb(model.get_part(nm))
        if aabb is None:
            continue
        if aabb[0][2] > 0.05:  # bottom should be near ground
            ok = False
    ctx.check("wheels_near_ground", ok, details=f"wheel parts={names}")


def run_tipping_barrow_tests(
    model: ArticulatedObject,
    config: TippingBarrowConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(model)

    _declare_overlaps(ctx, model)
    _declare_islands(ctx, model, r)

    ctx.check_model_valid()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.020)
    ctx.fail_if_joint_mating_has_gap()

    _expect_defining_joint(ctx, model, r)
    _expect_wheel_count(ctx, model, r)
    _expect_wheels_grounded(ctx, model, r)

    return ctx.report()


__all__ = [
    "TippingBarrowConfig",
    "ResolvedTippingBarrowConfig",
    "TippingBarrowPalette",
    "TubShapeModule",
    "TipMechanismModule",
    "WheelCountModule",
    "SupportModule",
    "build_tipping_barrow",
    "build_seeded_tipping_barrow",
    "config_from_seed",
    "resolve_config",
    "run_tipping_barrow_tests",
    "slot_choices_for_seed",
]
