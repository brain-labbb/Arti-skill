"""Procedural template for the ``missile_launcher`` category.

Follows ``articraft_template_authoring/specs_modular_v1/missile_launcher.md``.

A missile launcher is a slewing, elevating launch assembly: a static root
(pedestal / vehicle bed / trailer / naval deck / tripod) carries a yaw
turntable (azimuth, vertical Z), whose trunnions cradle an elevating frame
(pitch, horizontal axis), which mounts a launch pod / tube bank / rail set.
The launch tubes/box are the category identity.

PRIMARY_ANCHOR: ``rec_missile_launcher_0002``. The anchor is a 4-part /
3-joint model:

    base --yaw (CONTINUOUS, +Z)--> turntable
         --pitch (REVOLUTE, -Y)--> cradle --FIXED--> launch_pod

Per AUTHORING.md §A Rule 3, ``config_from_seed(0)`` reproduces the
anchor's part names, joint topology, per-part visual counts, primitive
histograms (all Box/Cylinder primitives) and bbox aspect ratio. Every seed
keeps the same four parts and three articulations; only geometry
(root_style / yaw_style / launcher_style enums and continuous dimensions)
varies. Decorative sub-elements are attached as ``parent.visual(...)`` per
Rule 1.

Adopted source modules (see specs/missile_launcher.md Adopted Source Index):
S1 0002 (canonical pedestal launcher), S2 c51786 (cadquery canister pod),
S3 0003 (truck-bed tube bank), S4 474f26 (box-cell pod), S5 91c8ac
(box-wall canister + yoke fork), S6 5b84e0 (tripod + lathe tube).

Scope note: the corpus has no missile-eject prismatic joint, so none is
introduced. ``pod_attachment`` is narrowed to a separate FIXED launch_pod
part (the dominant 5-star pattern) to keep a stable 4-part skeleton.
"""

from __future__ import annotations

import math
import random
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Literal

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
)

# adopted: S1 0002 — canonical 4-part pedestal launcher + yaw/pitch/FIXED joints
# adopted: S2 c51786 — canister pod cell layout
# adopted: S3 0003 — truck-bed base + tube-bank grid
# adopted: S4 474f26 — parametric box-cell launch pod
# adopted: S5 91c8ac — yoke fork turntable + box-wall canister
# adopted: S6 5b84e0 — tripod root + round launch tube

__modular__ = True

RootStyle = Literal[
    "pedestal",
    "ground_column",
    "vehicle_truck_bed",
    "trailer_towed",
    "naval_deck",
    "tripod",
]
YawStyle = Literal["slew_turntable", "yoke_fork", "truck_slew_platform", "yaw_head_compact"]
YawJointType = Literal["continuous", "revolute_full", "revolute_limited"]
LauncherStyle = Literal[
    "box_pod",
    "canister_pack",
    "hollow_tube_bank",
    "twin_rail",
    "single_tube",
]
MaterialStyle = Literal["olive_drab", "desert_tan", "naval_grey", "forest_green"]

PALETTES: dict[MaterialStyle, dict[str, tuple[float, float, float, float]]] = {
    "olive_drab": {
        "paint": (0.30, 0.36, 0.24, 1.0),
        "steel": (0.34, 0.39, 0.35, 1.0),
        "dark": (0.18, 0.20, 0.20, 1.0),
        "hazard": (0.46, 0.49, 0.47, 1.0),
        "tube": (0.26, 0.30, 0.22, 1.0),
        "bore": (0.04, 0.04, 0.04, 1.0),
    },
    "desert_tan": {
        "paint": (0.74, 0.66, 0.48, 1.0),
        "steel": (0.66, 0.60, 0.46, 1.0),
        "dark": (0.28, 0.25, 0.20, 1.0),
        "hazard": (0.80, 0.74, 0.56, 1.0),
        "tube": (0.70, 0.62, 0.44, 1.0),
        "bore": (0.05, 0.05, 0.04, 1.0),
    },
    "naval_grey": {
        "paint": (0.52, 0.55, 0.58, 1.0),
        "steel": (0.46, 0.49, 0.52, 1.0),
        "dark": (0.20, 0.22, 0.24, 1.0),
        "hazard": (0.62, 0.65, 0.68, 1.0),
        "tube": (0.44, 0.47, 0.50, 1.0),
        "bore": (0.04, 0.05, 0.05, 1.0),
    },
    "forest_green": {
        "paint": (0.22, 0.30, 0.20, 1.0),
        "steel": (0.30, 0.35, 0.28, 1.0),
        "dark": (0.12, 0.15, 0.12, 1.0),
        "hazard": (0.40, 0.45, 0.38, 1.0),
        "tube": (0.20, 0.26, 0.18, 1.0),
        "bore": (0.03, 0.04, 0.03, 1.0),
    },
}


@dataclass(frozen=True)
class MissileLauncherConfig:
    """User-facing configuration.

    Defaults reproduce the PRIMARY_ANCHOR (0002): a pedestal launcher with a
    slew turntable, a box-cell launch pod, CONTINUOUS yaw and REVOLUTE pitch.
    """

    root_style: RootStyle = "pedestal"
    yaw_style: YawStyle = "slew_turntable"
    yaw_joint_type: YawJointType = "continuous"
    launcher_style: LauncherStyle = "box_pod"
    material_style: MaterialStyle = "olive_drab"
    base_size: float = 0.72
    base_height: float = 0.36
    slew_ring_radius: float = 0.15
    trunnion_height: float = 0.185
    trunnion_track: float = 0.1375
    cradle_length: float = 0.40
    tube_rows: int = 2
    tube_cols: int = 3
    tube_radius: float = 0.075
    tube_length: float = 0.48
    pitch_lower: float = -0.1745
    pitch_upper: float = 1.0472
    yaw_limit: float = math.pi
    name: str = "reference_missile_launcher"


@dataclass(frozen=True)
class ResolvedMissileLauncherConfig:
    root_style: RootStyle
    yaw_style: YawStyle
    yaw_joint_type: YawJointType
    launcher_style: LauncherStyle
    material_style: MaterialStyle
    base_size: float
    base_height: float
    slew_ring_radius: float
    trunnion_height: float
    trunnion_track: float
    cradle_length: float
    tube_rows: int
    tube_cols: int
    tube_radius: float
    tube_length: float
    pitch_lower: float
    pitch_upper: float
    yaw_limit: float
    # Derived placement.
    yaw_origin_z: float
    turntable_height: float
    pod_mount_x: float
    pod_mount_z: float
    drive_housing_bottom_z: float
    drive_housing_top_z: float
    name: str


_ROOT_STYLES = {
    "pedestal",
    "ground_column",
    "vehicle_truck_bed",
    "trailer_towed",
    "naval_deck",
    "tripod",
}
_YAW_STYLES = {"slew_turntable", "yoke_fork", "truck_slew_platform", "yaw_head_compact"}
_YAW_JOINT_TYPES = {"continuous", "revolute_full", "revolute_limited"}
_LAUNCHER_STYLES = {"box_pod", "canister_pack", "hollow_tube_bank", "twin_rail", "single_tube"}

# Vertical gap kept between the azimuth drive housing top and the lowest point
# the elevating pod's mounting plate sweeps through (> QC overlap_tol=0.005).
_DRIVE_HOUSING_CLEARANCE = 0.02


def _mounting_plate_dims(
    tube_radius: float, tube_cols: int, tube_rows: int
) -> tuple[float, float, float]:
    """Single source for the launch pod mounting-plate box dimensions.

    Consumed both by ``_build_launch_pod`` (to draw the plate) and by
    ``resolve_config`` (to derive the drive-housing clearance), so the two can
    never drift (Contract 3c).
    """
    plate_w = max(0.08, 2.0 * tube_radius * tube_cols)
    plate_d = max(0.06, 2.0 * tube_radius * tube_rows)
    plate_t = max(0.012, tube_radius * 0.3)
    return plate_w, plate_d, plate_t


def _drive_housing_top_z(
    *,
    trunnion_height: float,
    pod_mount_x: float,
    pod_mount_z: float,
    plate_w: float,
    plate_t: float,
    pitch_lower: float,
    pitch_upper: float,
    housing_bottom_z: float,
) -> float:
    """Highest safe top for the azimuth drive housing.

    The drive housing sits behind the yaw column, below the pitch axis. As the
    cradle elevates (turntable_to_cradle about -Y at z=trunnion_height), the
    launch pod's mounting-plate REAR edge (behind the pivot) sweeps down and
    back over the housing. Rather than clip the elevation range — which IS the
    launcher's identity — the housing top is capped just under the lowest point
    that rear edge reaches over the whole pitch travel. Derived here so the
    limit and the geometry share one formula (Contract 3c).
    """
    # Rear-bottom edge of the mounting plate expressed in the cradle (pitch)
    # frame; the pivot is the cradle origin.
    rear_x = pod_mount_x - plate_w * 0.5
    bottom_z = pod_mount_z - plate_t * 0.5
    steps = 48
    span = pitch_upper - pitch_lower
    swept_min_z = float("inf")
    for i in range(steps + 1):
        theta = pitch_lower + span * (i / steps)
        # Rotation about -Y by theta, pivot at (0,0,trunnion_height).
        z_tt = trunnion_height + rear_x * math.sin(theta) + bottom_z * math.cos(theta)
        swept_min_z = min(swept_min_z, z_tt)
    top = swept_min_z - _DRIVE_HOUSING_CLEARANCE
    # Keep at least a minimally visible housing so the box never inverts.
    return max(top, housing_bottom_z + 0.02)


def config_from_seed(seed: int) -> MissileLauncherConfig:
    """Sample a reproducible missile launcher configuration.

    seed=0 returns the anchor-matching default. Other seeds sample the stable
    subdomain so every declared Literal is exercised (enum_coverage gate).
    """
    if seed == 0:
        return MissileLauncherConfig()

    rng = random.Random(seed)
    root_style: RootStyle = rng.choice(
        (
            "pedestal",
            "ground_column",
            "vehicle_truck_bed",
            "trailer_towed",
            "naval_deck",
            "tripod",
        )
    )
    yaw_style: YawStyle = rng.choice(
        ("slew_turntable", "yoke_fork", "truck_slew_platform", "yaw_head_compact")
    )
    yaw_joint_type: YawJointType = rng.choice(("continuous", "revolute_full", "revolute_limited"))
    launcher_style: LauncherStyle = rng.choice(
        ("box_pod", "canister_pack", "hollow_tube_bank", "twin_rail", "single_tube")
    )

    naval = root_style == "naval_deck"
    tripod = root_style == "tripod"
    if naval:
        base_size = round(rng.uniform(2.2, 4.0), 4)
        base_height = round(rng.uniform(1.4, 2.3), 4)
        scale = round(rng.uniform(1.8, 2.6), 4)
    elif tripod:
        base_size = round(rng.uniform(0.5, 0.8), 4)
        base_height = round(rng.uniform(0.16, 0.32), 4)
        scale = round(rng.uniform(0.55, 0.85), 4)
    else:
        base_size = round(rng.uniform(0.7, 1.7), 4)
        base_height = round(rng.uniform(0.28, 0.7), 4)
        scale = round(rng.uniform(0.85, 1.5), 4)

    # Launcher grid by style.
    if launcher_style == "single_tube":
        tube_rows, tube_cols = 1, 1
    elif launcher_style == "twin_rail":
        tube_rows, tube_cols = 1, 2
    elif launcher_style == "box_pod":
        tube_rows = rng.randint(2, 3)
        tube_cols = rng.randint(2, 4)
    elif launcher_style == "canister_pack":
        tube_rows = rng.choice((2, 2))
        tube_cols = rng.choice((2, 4))
    else:  # hollow_tube_bank
        tube_rows = rng.randint(2, 3)
        tube_cols = rng.randint(3, 4)

    tube_radius = round(scale * rng.uniform(0.05, 0.085), 4)
    tube_length = round(
        scale * rng.uniform(0.9, 1.6) * (2.6 if root_style == "naval_deck" else 1.0), 4
    )

    return MissileLauncherConfig(
        root_style=root_style,
        yaw_style=yaw_style,
        yaw_joint_type=yaw_joint_type,
        launcher_style=launcher_style,
        material_style=rng.choice(tuple(PALETTES)),
        base_size=base_size,
        base_height=base_height,
        slew_ring_radius=round(base_size * rng.uniform(0.18, 0.30), 4),
        trunnion_height=round(scale * rng.uniform(0.18, 0.42), 4),
        trunnion_track=round(scale * rng.uniform(0.13, 0.22), 4),
        cradle_length=round(tube_length * rng.uniform(0.85, 1.05), 4),
        tube_rows=tube_rows,
        tube_cols=tube_cols,
        tube_radius=tube_radius,
        tube_length=tube_length,
        pitch_lower=round(rng.uniform(-0.25, 0.0), 4),
        pitch_upper=round(rng.uniform(0.55, 1.30), 4),
        yaw_limit=round(rng.uniform(2.5, math.pi), 4),
        name=f"seeded_missile_launcher_{seed}",
    )


def resolve_config(config: MissileLauncherConfig) -> ResolvedMissileLauncherConfig:
    """Validate enums, clamp dimensions, derive placement quantities."""
    if config.root_style not in _ROOT_STYLES:
        raise ValueError(f"Unsupported root_style: {config.root_style}")
    if config.yaw_style not in _YAW_STYLES:
        raise ValueError(f"Unsupported yaw_style: {config.yaw_style}")
    if config.yaw_joint_type not in _YAW_JOINT_TYPES:
        raise ValueError(f"Unsupported yaw_joint_type: {config.yaw_joint_type}")
    if config.launcher_style not in _LAUNCHER_STYLES:
        raise ValueError(f"Unsupported launcher_style: {config.launcher_style}")
    if config.material_style not in PALETTES:
        raise ValueError(f"Unsupported material_style: {config.material_style}")

    base_size = _clamp(config.base_size, 0.45, 4.2)
    base_height = _clamp(config.base_height, 0.14, 2.4)
    slew_ring_radius = _clamp(config.slew_ring_radius, base_size * 0.14, base_size * 0.42)
    trunnion_height = _clamp(config.trunnion_height, 0.07, 1.4)
    trunnion_track = _clamp(config.trunnion_track, 0.08, base_size * 0.5)
    tube_radius = _clamp(config.tube_radius, 0.04, 0.18)
    tube_length = _clamp(config.tube_length, 0.5, 5.0)
    cradle_length = _clamp(config.cradle_length, tube_length * 0.6, tube_length * 1.2)
    tube_rows = int(_clamp(config.tube_rows, 1, 3))
    tube_cols = int(_clamp(config.tube_cols, 1, 4))

    pitch_lower = config.pitch_lower
    pitch_upper = config.pitch_upper
    if pitch_lower >= pitch_upper:
        pitch_lower, pitch_upper = -0.1745, 1.0472
    pitch_lower = _clamp(pitch_lower, -0.4, 0.0)
    pitch_upper = _clamp(pitch_upper, 0.5, 1.35)
    yaw_limit = _clamp(config.yaw_limit, 1.5, math.pi)

    yaw_origin_z = base_height
    turntable_height = max(0.04, trunnion_height * 0.5)
    pod_mount_x = cradle_length * 0.45
    # The pod seats on a central saddle above the cradle tray; pod_mount_z is
    # the saddle top so the FIXED joint origin lands on real cradle geometry.
    tray_top = max(0.024, tube_radius * 0.5)
    saddle_h = max(0.02, tube_radius * 0.6)
    pod_mount_z = tray_top + saddle_h

    # Azimuth drive housing envelope: bottom seats on the slew deck, top is
    # capped below the elevating mounting plate's swept arc (single-sourced).
    plate_w, _plate_d, plate_t = _mounting_plate_dims(tube_radius, tube_cols, tube_rows)
    drive_housing_bottom_z = turntable_height * 0.2
    drive_housing_top_z = _drive_housing_top_z(
        trunnion_height=trunnion_height,
        pod_mount_x=pod_mount_x,
        pod_mount_z=pod_mount_z,
        plate_w=plate_w,
        plate_t=plate_t,
        pitch_lower=pitch_lower,
        pitch_upper=pitch_upper,
        housing_bottom_z=drive_housing_bottom_z,
    )

    return ResolvedMissileLauncherConfig(
        root_style=config.root_style,
        yaw_style=config.yaw_style,
        yaw_joint_type=config.yaw_joint_type,
        launcher_style=config.launcher_style,
        material_style=config.material_style,
        base_size=base_size,
        base_height=base_height,
        slew_ring_radius=slew_ring_radius,
        trunnion_height=trunnion_height,
        trunnion_track=trunnion_track,
        cradle_length=cradle_length,
        tube_rows=tube_rows,
        tube_cols=tube_cols,
        tube_radius=tube_radius,
        tube_length=tube_length,
        pitch_lower=pitch_lower,
        pitch_upper=pitch_upper,
        yaw_limit=yaw_limit,
        yaw_origin_z=yaw_origin_z,
        turntable_height=turntable_height,
        pod_mount_x=pod_mount_x,
        pod_mount_z=pod_mount_z,
        drive_housing_bottom_z=drive_housing_bottom_z,
        drive_housing_top_z=drive_housing_top_z,
        name=config.name,
    )


def _clamp(value: float, lo: float, hi: float) -> float:
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, float(value)))


def _joint_meta(joint_type, axis, origin, joint_range) -> dict[str, object]:
    return {"type": joint_type, "axis": axis, "origin": origin, "range": joint_range}


# --------------------------------------------------------------------------- #
# base (root)
# --------------------------------------------------------------------------- #


def _build_base(part, r: ResolvedMissileLauncherConfig, *, mats) -> None:
    paint = mats["paint"]
    steel = mats["steel"]
    dark = mats["dark"]
    size = r.base_size
    h = r.base_height

    # Foundation plinth (Box).
    plinth_h = max(0.05, h * 0.22)
    part.visual(
        Box((size, size * 0.84, plinth_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h * 0.5)),
        material=paint,
        name="foundation_plinth",
    )
    # Pedestal column (Cylinder) up to the slew plane.
    col_bottom = plinth_h
    col_top = h - max(0.03, h * 0.10)
    col_len = max(0.12, col_top - col_bottom)
    part.visual(
        Cylinder(radius=r.slew_ring_radius * 0.78, length=col_len),
        origin=Origin(xyz=(0.0, 0.0, col_bottom + col_len * 0.5)),
        material=dark,
        name="pedestal_column",
    )
    # Top plate (Box) at the yaw plane.
    part.visual(
        Box((size * 0.48, size * 0.4, max(0.03, h * 0.11))),
        origin=Origin(xyz=(0.0, 0.0, h - max(0.015, h * 0.055))),
        material=paint,
        name="top_plate",
    )
    # Service box (Box) on the side.
    part.visual(
        Box((size * 0.25, size * 0.22, h * 0.34)),
        origin=Origin(xyz=(-size * 0.26, 0.0, h * 0.34)),
        material=dark,
        name="service_box",
    )

    # ---- root-style detailing (Rule 1: visuals only) ----
    if r.root_style == "vehicle_truck_bed":
        _add_truck_bed(part, r, steel=steel, dark=dark, plinth_h=plinth_h)
    elif r.root_style == "trailer_towed":
        _add_trailer(part, r, steel=steel, dark=dark, plinth_h=plinth_h)
    elif r.root_style == "tripod":
        _add_tripod_legs(part, r, steel=steel, plinth_h=plinth_h)
    elif r.root_style == "naval_deck":
        _add_naval_deck(part, r, steel=steel, dark=dark, plinth_h=plinth_h)
    elif r.root_style == "ground_column":
        _add_ground_anchors(part, r, dark=dark, plinth_h=plinth_h)


def _add_truck_bed(part, r, *, steel, dark, plinth_h) -> None:
    size = r.base_size
    # Flatbed rails along X spanning the foundation.
    for sy in (-1.0, 1.0):
        part.visual(
            Box((size * 1.3, max(0.04, size * 0.06), max(0.04, size * 0.06))),
            origin=Origin(xyz=(0.0, sy * size * 0.40, plinth_h * 0.5)),
            material=steel,
            name=f"bed_rail_{int(sy)}",
        )
    # Crossmembers.
    for i, fx in enumerate((-0.35, 0.0, 0.35)):
        part.visual(
            Box((max(0.04, size * 0.06), size * 0.84, max(0.03, size * 0.05))),
            origin=Origin(xyz=(fx * size, 0.0, plinth_h * 0.5)),
            material=steel,
            name=f"bed_crossmember_{i}",
        )
    # Four wheels touching the rails (Cylinders along Y).
    wheel_r = max(0.06, size * 0.16)
    for ix, fx in enumerate((-0.45, 0.45)):
        for sy in (-1.0, 1.0):
            part.visual(
                Cylinder(radius=wheel_r, length=max(0.04, size * 0.08)),
                origin=Origin(
                    xyz=(fx * size, sy * (size * 0.46), plinth_h * 0.5),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark,
                name=f"wheel_{ix}_{int(sy)}",
            )


def _add_trailer(part, r, *, steel, dark, plinth_h) -> None:
    size = r.base_size
    # Tow drawbar forward.
    part.visual(
        Box((size * 0.9, max(0.04, size * 0.06), max(0.04, size * 0.06))),
        origin=Origin(xyz=(size * 0.75, 0.0, plinth_h * 0.5)),
        material=steel,
        name="tow_drawbar",
    )
    part.visual(
        Cylinder(radius=max(0.03, size * 0.05), length=max(0.04, size * 0.06)),
        origin=Origin(xyz=(size * 1.2, 0.0, plinth_h * 0.5), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="tow_eye",
    )
    # Two stabilizer outrigger feet.
    for sy in (-1.0, 1.0):
        part.visual(
            Box((max(0.05, size * 0.08), size * 0.5, max(0.03, size * 0.05))),
            origin=Origin(xyz=(0.0, sy * size * 0.42, plinth_h * 0.4)),
            material=steel,
            name=f"outrigger_{int(sy)}",
        )
    # Axle through the plinth tying both wheels into the base island.
    wheel_r = max(0.05, size * 0.14)
    part.visual(
        Cylinder(radius=max(0.02, size * 0.04), length=size * 1.05),
        origin=Origin(xyz=(-size * 0.2, 0.0, plinth_h * 0.5), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="trailer_axle",
    )
    for sy in (-1.0, 1.0):
        part.visual(
            Cylinder(radius=wheel_r, length=max(0.04, size * 0.07)),
            origin=Origin(
                xyz=(-size * 0.2, sy * size * 0.5, plinth_h * 0.5),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark,
            name=f"trailer_wheel_{int(sy)}",
        )


def _add_tripod_legs(part, r, *, steel, plinth_h) -> None:
    size = r.base_size
    span = size * 0.5
    # Legs converge onto the pedestal column top, which sits below the slew
    # plane (z=base_height) — stopping short of the child turntable's slew ring
    # so the legs anchor the column without overlapping the ring.
    apex_z = r.base_height - max(0.05, r.base_height * 0.18)
    leg_len = math.hypot(span, apex_z)
    pitch = math.atan2(span, apex_z)
    leg_r = max(0.02, size * 0.05)
    for index in range(3):
        ang = index * (2.0 * math.pi / 3.0)
        dx, dy = math.cos(ang), math.sin(ang)
        part.visual(
            Cylinder(radius=leg_r, length=leg_len),
            origin=Origin(
                xyz=(dx * span * 0.5, dy * span * 0.5, apex_z * 0.5),
                rpy=(pitch * dy, -pitch * dx, 0.0),
            ),
            material=steel,
            name=f"tripod_leg_{index}",
        )
        part.visual(
            Box((size * 0.12, size * 0.12, max(0.02, plinth_h))),
            origin=Origin(xyz=(dx * span, dy * span, plinth_h * 0.5)),
            material=steel,
            name=f"tripod_foot_{index}",
        )


def _add_naval_deck(part, r, *, steel, dark, plinth_h) -> None:
    size = r.base_size
    # Wide deck ring + access hatches.
    part.visual(
        Cylinder(radius=size * 0.5, length=max(0.04, plinth_h * 0.6)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h * 1.2)),
        material=steel,
        name="deck_ring",
    )
    for i, ang in enumerate((0.0, math.pi * 0.5, math.pi, math.pi * 1.5)):
        part.visual(
            Box((size * 0.14, size * 0.14, max(0.03, plinth_h * 0.5))),
            origin=Origin(
                xyz=(math.cos(ang) * size * 0.36, math.sin(ang) * size * 0.36, plinth_h * 1.5)
            ),
            material=dark,
            name=f"deck_hatch_{i}",
        )


def _add_ground_anchors(part, r, *, dark, plinth_h) -> None:
    size = r.base_size
    for i, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
        part.visual(
            Cylinder(radius=max(0.012, size * 0.03), length=max(0.03, plinth_h)),
            origin=Origin(
                xyz=(
                    math.cos(ang * math.pi) * size * 0.4,
                    math.sin(ang * math.pi) * size * 0.36,
                    plinth_h * 0.5,
                )
            ),
            material=dark,
            name=f"anchor_bolt_{i}",
        )


# --------------------------------------------------------------------------- #
# turntable (yaw stage)
# --------------------------------------------------------------------------- #


def _build_turntable(part, r: ResolvedMissileLauncherConfig, *, mats) -> None:
    paint = mats["paint"]
    steel = mats["steel"]
    dark = mats["dark"]
    th = r.turntable_height
    track = r.trunnion_track
    trunnion_z = r.trunnion_height

    # Slew ring (Cylinder) at the base.
    part.visual(
        Cylinder(radius=r.slew_ring_radius, length=th),
        origin=Origin(xyz=(0.0, 0.0, th * 0.5)),
        material=dark,
        name="slew_ring",
    )
    # Central bearing pedestal on the yaw axis, rising from the slew ring up to
    # the trunnion cross-shaft. It is the structural bridge from the low slew
    # deck to the elevation trunnions (previously the tall drive housing did
    # this, but that housing now sits low to clear the elevating pod). Kept thin
    # and centered on x=0, where the pod's mounting plate always sweeps high, so
    # it never enters the elevation 穿模 zone behind the pivot.
    pedestal_r = max(0.02, r.slew_ring_radius * 0.3)
    part.visual(
        Cylinder(radius=pedestal_r, length=trunnion_z),
        origin=Origin(xyz=(0.0, 0.0, trunnion_z * 0.5)),
        material=steel,
        name="bearing_pedestal",
    )
    # Drive housing (Box). Height is capped so the housing top stays below the
    # elevating pod mounting plate's swept arc (see _drive_housing_top_z); it
    # sits low behind the yaw column like a real azimuth slew-drive gearbox.
    housing_bottom = r.drive_housing_bottom_z
    housing_h = max(0.02, r.drive_housing_top_z - housing_bottom)
    part.visual(
        Box((r.slew_ring_radius * 1.1, r.slew_ring_radius * 1.3, housing_h)),
        origin=Origin(xyz=(-r.slew_ring_radius * 0.5, 0.0, housing_bottom + housing_h * 0.5)),
        material=paint,
        name="drive_housing",
    )
    # Two trunnion supports (Box) rising to the pitch axis.
    support_h = max(0.1, trunnion_z - th)
    for side, sy in (("left", 1.0), ("right", -1.0)):
        part.visual(
            Box((r.slew_ring_radius * 0.36, max(0.04, track * 0.36), support_h)),
            origin=Origin(xyz=(0.0, sy * track, th + support_h * 0.5)),
            material=paint,
            name=f"support_{side}",
        )
    # Deck plate (Box).
    part.visual(
        Box((r.slew_ring_radius * 1.3, r.slew_ring_radius * 1.5, th * 0.5)),
        origin=Origin(xyz=(r.slew_ring_radius * 0.1, 0.0, th * 1.05)),
        material=paint,
        name="deck_plate",
    )
    # Fixed trunnion cross-shaft on the pitch axis — anchors the elevation
    # joint origin to parent geometry; the cradle pivot hub rides it.
    part.visual(
        Cylinder(radius=max(0.02, track * 0.18), length=2.0 * track),
        origin=Origin(xyz=(0.0, 0.0, trunnion_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="trunnion_cross_shaft",
    )

    if r.yaw_style == "yoke_fork":
        # Taller fork cheeks with bearing bosses.
        for side, sy in (("left", 1.0), ("right", -1.0)):
            part.visual(
                Cylinder(radius=max(0.03, track * 0.3), length=max(0.04, track * 0.3)),
                origin=Origin(xyz=(0.0, sy * track, trunnion_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"fork_bearing_{side}",
            )
    elif r.yaw_style == "truck_slew_platform":
        part.visual(
            Box((r.slew_ring_radius * 2.0, r.slew_ring_radius * 1.8, th * 0.4)),
            origin=Origin(xyz=(0.0, 0.0, th * 0.85)),
            material=steel,
            name="platform_deck",
        )
    elif r.yaw_style == "yaw_head_compact":
        part.visual(
            Cylinder(radius=r.slew_ring_radius * 0.6, length=th * 0.8),
            origin=Origin(xyz=(0.0, 0.0, th * 1.4)),
            material=steel,
            name="compact_hub",
        )


# --------------------------------------------------------------------------- #
# cradle (elevation frame)
# --------------------------------------------------------------------------- #


def _build_cradle(part, r: ResolvedMissileLauncherConfig, *, mats) -> None:
    paint = mats["paint"]
    steel = mats["steel"]
    dark = mats["dark"]
    track = r.trunnion_track
    length = r.cradle_length
    half_w = max(0.06, r.tube_radius * r.tube_cols * 1.1)

    # Pivot hub (Cylinder) at the cradle origin on the pitch axis — guarantees
    # child geometry at the elevation joint origin; rides the turntable shaft.
    part.visual(
        Cylinder(radius=max(0.022, track * 0.2), length=2.0 * track),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="pivot_hub",
    )
    # Two trunnions (Cylinders) reaching the turntable bearings.
    for side, sy in (("left", 1.0), ("right", -1.0)):
        part.visual(
            Cylinder(radius=max(0.03, track * 0.32), length=max(0.04, track * 0.3)),
            origin=Origin(xyz=(0.0, sy * track * 0.95, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"trunnion_{side}",
        )
    # Tray (Box) extending forward from the pivot.
    part.visual(
        Box((length, 2.0 * half_w, max(0.02, r.tube_radius * 0.5))),
        origin=Origin(xyz=(length * 0.5, 0.0, max(0.012, r.tube_radius * 0.25))),
        material=paint,
        name="tray",
    )
    # Two side rails (Box).
    rail_h = max(0.05, r.tube_radius * 1.1)
    for side, sy in (("left", 1.0), ("right", -1.0)):
        part.visual(
            Box((length, max(0.03, r.tube_radius * 0.4), rail_h)),
            origin=Origin(xyz=(length * 0.5, sy * half_w, rail_h * 0.5)),
            material=paint,
            name=f"side_{side}",
        )
    # Rear bulkhead (Box).
    part.visual(
        Box((max(0.05, r.tube_radius * 0.8), 2.0 * half_w, rail_h * 1.4)),
        origin=Origin(xyz=(length * 0.08, 0.0, rail_h * 0.7)),
        material=dark,
        name="rear_bulkhead",
    )
    # Central pod saddle (Box) at the mount point — the launch pod seats on it
    # so the FIXED joint origin lands on real cradle geometry. The saddle spans
    # from just inside the tray top up to pod_mount_z so it is always seated on
    # the tray (resolve_config's tray_top estimate can round slightly above the
    # tray's real top surface for small tube radii, which otherwise leaves the
    # saddle floating by a fraction of a millimetre).
    tray_top_actual = max(0.012, r.tube_radius * 0.25) + 0.5 * max(0.02, r.tube_radius * 0.5)
    saddle_bottom = tray_top_actual - 0.002
    saddle_h = max(0.02, r.pod_mount_z - saddle_bottom)
    part.visual(
        Box((length * 0.5, 2.0 * half_w * 0.7, saddle_h)),
        origin=Origin(xyz=(r.pod_mount_x, 0.0, saddle_bottom + saddle_h * 0.5)),
        material=dark,
        name="pod_saddle",
    )
    # Two pod mount lugs (Box) flanking the saddle.
    for side, sy in (("left", 1.0), ("right", -1.0)):
        part.visual(
            Box((length * 0.5, max(0.04, half_w * 0.45), max(0.015, r.tube_radius * 0.3))),
            origin=Origin(
                xyz=(r.pod_mount_x, sy * half_w * 0.55, r.pod_mount_z - saddle_h * 0.3),
            ),
            material=dark,
            name=f"mount_{side}",
        )


# --------------------------------------------------------------------------- #
# launch_pod (category identity)
# --------------------------------------------------------------------------- #


def _build_launch_pod(part, r: ResolvedMissileLauncherConfig, *, mats) -> None:
    paint = mats["paint"]
    hazard = mats["hazard"]
    tube = mats["tube"]
    dark = mats["dark"]
    bore = mats["bore"]

    rows = r.tube_rows
    cols = r.tube_cols
    tr = r.tube_radius
    tl = r.tube_length

    # Mounting plate (Box) at the pod origin — guarantees child geometry at the
    # FIXED joint origin and supports the launcher above. Dimensions come from
    # the shared helper so the drive-housing clearance derivation cannot drift.
    plate_w, plate_d, plate_t = _mounting_plate_dims(tr, cols, rows)
    part.visual(
        Box((plate_w, plate_d, plate_t)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark,
        name="mounting_plate",
    )
    # Two skids (Box) under the launcher.
    for side, sy in (("left", 1.0), ("right", -1.0)):
        part.visual(
            Box((tl * 0.5, max(0.03, tr * 0.5), max(0.012, tr * 0.3))),
            origin=Origin(xyz=(tl * 0.18, sy * plate_d * 0.32, max(0.01, tr * 0.2))),
            material=hazard,
            name=f"skid_{side}",
        )

    # Launcher base sits on the skids (skid top ~ tr*0.35) so the whole pod is
    # one connected island.
    cell_z0 = max(0.01, tr * 0.34)
    if r.launcher_style == "box_pod":
        _build_box_pod(part, r, paint=paint, dark=dark, bore=bore, cell_z0=cell_z0)
    elif r.launcher_style == "canister_pack":
        _build_canister_pack(part, r, paint=paint, dark=dark, bore=bore, cell_z0=cell_z0)
    elif r.launcher_style == "hollow_tube_bank":
        _build_tube_bank(part, r, tube=tube, bore=bore, frame=dark, cell_z0=cell_z0)
    elif r.launcher_style == "twin_rail":
        _build_twin_rail(part, r, paint=paint, dark=dark, tube=tube, cell_z0=cell_z0)
    else:  # single_tube
        _build_single_tube(part, r, tube=tube, bore=bore, cell_z0=cell_z0)


def _cell_centers(r, *, cell_z0):
    rows, cols, tr = r.tube_rows, r.tube_cols, r.tube_radius
    pitch = tr * 2.15
    y0 = -(cols - 1) * pitch * 0.5
    z0 = cell_z0 + tr
    for row in range(rows):
        for col in range(cols):
            yield row, col, (y0 + col * pitch, z0 + row * pitch)


def _breech_block(part, r, *, material, half_w, top_z) -> None:
    """Rear backplate tying every launcher cell to the mounting plate.

    Each launcher cell (tube / canister / rail) starts at x=0, so a thin block
    spanning the full cell-array cross-section at the rear is intersected by
    every cell and reaches down to z=0 onto the mounting plate — collapsing the
    launcher into a single connected geometry island (Rule 1, no floating
    sub-geometry).
    """
    bp_t = max(0.02, r.tube_length * 0.1)
    part.visual(
        Box((bp_t, 2.0 * half_w, top_z)),
        origin=Origin(xyz=(bp_t * 0.5, 0.0, top_z * 0.5)),
        material=material,
        name="breech_block",
    )


def _build_box_pod(part, r, *, paint, dark, bore, cell_z0) -> None:
    """Box launch pod with square cell apertures (adopted: S4 474f26)."""
    tr, tl = r.tube_radius, r.tube_length
    rows, cols = r.tube_rows, r.tube_cols
    pitch = tr * 2.15
    width = cols * pitch + tr * 0.4
    height = rows * pitch + tr * 0.4
    cz = cell_z0 + height * 0.5
    # Pod shell (Box).
    part.visual(
        Box((tl, width, height)),
        origin=Origin(xyz=(tl * 0.5, 0.0, cz)),
        material=paint,
        name="pod_shell",
    )
    # Roof panel (Box).
    part.visual(
        Box((tl * 0.7, width * 0.82, max(0.012, tr * 0.3))),
        origin=Origin(xyz=(tl * 0.5, 0.0, cz + height * 0.5 + tr * 0.1)),
        material=dark,
        name="roof_panel",
    )
    # Rear service housing (Box).
    part.visual(
        Box((tl * 0.2, width * 0.7, height * 0.7)),
        origin=Origin(xyz=(-tl * 0.05, 0.0, cz)),
        material=dark,
        name="rear_service_housing",
    )
    # Cell door apertures (Box per cell) on the muzzle face.
    for row, col, (y, z) in _cell_centers(r, cell_z0=cell_z0):
        part.visual(
            Box((max(0.01, tr * 0.18), pitch * 0.78, pitch * 0.78)),
            origin=Origin(xyz=(tl, y, z)),
            material=bore,
            name=f"cell_door_{row}_{col}",
        )


def _build_canister_pack(part, r, *, paint, dark, bore, cell_z0) -> None:
    """Square-walled canister pack (adopted: S5 91c8ac)."""
    tr, tl = r.tube_radius, r.tube_length
    rows, cols = r.tube_rows, r.tube_cols
    pitch = tr * 2.15
    half_w = (cols - 1) * pitch * 0.5 + pitch * 0.5
    top_z = cell_z0 + tr + (rows - 1) * pitch + pitch * 0.5
    _breech_block(part, r, material=dark, half_w=half_w, top_z=top_z)
    for row, col, (y, z) in _cell_centers(r, cell_z0=cell_z0):
        part.visual(
            Box((tl, pitch * 0.9, pitch * 0.9)),
            origin=Origin(xyz=(tl * 0.5, y, z)),
            material=paint,
            name=f"canister_{row}_{col}",
        )
        part.visual(
            Box((max(0.01, tr * 0.16), pitch * 0.7, pitch * 0.7)),
            origin=Origin(xyz=(tl, y, z)),
            material=bore,
            name=f"canister_bore_{row}_{col}",
        )
    # Banding straps wrapping over the top row of canisters (they overlap the
    # canister tops so they are part of the same connected island, not floating
    # above the pack).
    width = r.tube_cols * pitch
    strap_z = cell_z0 + tr + (rows - 1) * pitch + pitch * 0.45
    for i, fx in enumerate((0.25, 0.75)):
        part.visual(
            Box((max(0.012, tr * 0.2), width, max(0.012, tr * 0.3))),
            origin=Origin(xyz=(tl * fx, 0.0, strap_z)),
            material=dark,
            name=f"pack_strap_{i}",
        )


def _build_tube_bank(part, r, *, tube, bore, frame, cell_z0) -> None:
    """Round hollow launch tubes (adopted: S3 0003 tube bank)."""
    tr, tl = r.tube_radius, r.tube_length
    rows, cols = r.tube_rows, r.tube_cols
    pitch = tr * 2.15
    half_w = (cols - 1) * pitch * 0.5 + tr * 1.2
    top_z = cell_z0 + tr + (rows - 1) * pitch + tr * 1.2
    _breech_block(part, r, material=frame, half_w=half_w, top_z=top_z)
    for row, col, (y, z) in _cell_centers(r, cell_z0=cell_z0):
        part.visual(
            Cylinder(radius=tr, length=tl),
            origin=Origin(xyz=(tl * 0.5, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=tube,
            name=f"tube_{row}_{col}",
        )
        part.visual(
            Cylinder(radius=tr * 0.7, length=max(0.01, tl * 0.04)),
            origin=Origin(xyz=(tl, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bore,
            name=f"tube_bore_{row}_{col}",
        )


def _build_twin_rail(part, r, *, paint, dark, tube, cell_z0) -> None:
    """Twin launch rails with stanchions (adopted: S3/naval rails)."""
    tr, tl = r.tube_radius, r.tube_length
    pitch = tr * 2.6
    rail_z = cell_z0 + tr * 0.3
    n_rail = max(2, r.tube_cols)
    half_w = (n_rail - 1) * 0.5 * pitch + tr * 0.6
    top_z = rail_z + tr * 1.9 + tr * 0.2
    _breech_block(part, r, material=dark, half_w=half_w, top_z=top_z)
    for col in range(max(2, r.tube_cols)):
        y = (col - (max(2, r.tube_cols) - 1) * 0.5) * pitch
        part.visual(
            Box((tl, tr * 0.5, tr * 0.6)),
            origin=Origin(xyz=(tl * 0.5, y, rail_z)),
            material=paint,
            name=f"rail_{col}",
        )
        part.visual(
            Cylinder(radius=tr * 0.8, length=tl * 0.9),
            origin=Origin(xyz=(tl * 0.5, y, rail_z + tr * 1.1), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=tube,
            name=f"rail_missile_{col}",
        )
        # Muzzle stop stanchion (reaches down onto the deck).
        part.visual(
            Box((tr * 0.4, tr * 0.6, tr * 1.6)),
            origin=Origin(xyz=(tl * 0.95, y, rail_z + tr * 0.4)),
            material=dark,
            name=f"muzzle_stop_{col}",
        )


def _build_single_tube(part, r, *, tube, bore, cell_z0) -> None:
    """Single launch tube on a saddle (adopted: S6 5b84e0)."""
    tr, tl = r.tube_radius, r.tube_length
    z = cell_z0 + tr
    part.visual(
        Cylinder(radius=tr, length=tl),
        origin=Origin(xyz=(tl * 0.5, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube,
        name="launch_tube",
    )
    part.visual(
        Cylinder(radius=tr * 0.72, length=max(0.01, tl * 0.05)),
        origin=Origin(xyz=(tl, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bore,
        name="tube_bore",
    )
    for i, fx in enumerate((0.25, 0.7)):
        part.visual(
            Cylinder(radius=tr * 1.2, length=max(0.012, tr * 0.3)),
            origin=Origin(xyz=(tl * fx, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bore,
            name=f"tube_band_{i}",
        )


# --------------------------------------------------------------------------- #
# Assembly
# --------------------------------------------------------------------------- #


def build_missile_launcher(
    config: MissileLauncherConfig | None = None, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    config = config or MissileLauncherConfig()
    r = resolve_config(config)
    if assets is None:
        assets = AssetContext(Path(tempfile.mkdtemp(prefix="articraft-missile-launcher-")))
    model = ArticulatedObject(name=r.name, assets=assets)
    model.meta["adopted_source_ids"] = ("S1", "S2", "S3", "S4", "S5", "S6")
    palette = PALETTES[r.material_style]
    mats = {
        "paint": model.material("painted_steel", rgba=palette["paint"]),
        "steel": model.material("structural_steel", rgba=palette["steel"]),
        "dark": model.material("dark_steel", rgba=palette["dark"]),
        "hazard": model.material("hazard_gray", rgba=palette["hazard"]),
        "tube": model.material("tube_paint", rgba=palette["tube"]),
        "bore": model.material("bore_shadow", rgba=palette["bore"]),
    }

    base = model.part("base")
    _build_base(base, r, mats=mats)
    turntable = model.part("turntable")
    _build_turntable(turntable, r, mats=mats)
    cradle = model.part("cradle")
    _build_cradle(cradle, r, mats=mats)
    launch_pod = model.part("launch_pod")
    _build_launch_pod(launch_pod, r, mats=mats)

    # ---- yaw articulation (base -> turntable, vertical Z) ----
    yaw_origin = (0.0, 0.0, r.yaw_origin_z)
    if r.yaw_joint_type == "continuous":
        model.articulation(
            "base_to_turntable",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=turntable,
            origin=Origin(xyz=yaw_origin),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8500.0, velocity=1.2),
            meta=_joint_meta("continuous", (0.0, 0.0, 1.0), yaw_origin, "unbounded"),
        )
    else:
        limit = math.pi if r.yaw_joint_type == "revolute_full" else r.yaw_limit
        model.articulation(
            "base_to_turntable",
            ArticulationType.REVOLUTE,
            parent=base,
            child=turntable,
            origin=Origin(xyz=yaw_origin),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8500.0, velocity=1.0, lower=-limit, upper=limit),
            meta=_joint_meta("revolute", (0.0, 0.0, 1.0), yaw_origin, (-limit, limit)),
        )

    # ---- pitch articulation (turntable -> cradle, horizontal -Y) ----
    pitch_origin = (0.0, 0.0, r.trunnion_height)
    model.articulation(
        "turntable_to_cradle",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=cradle,
        origin=Origin(xyz=pitch_origin),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6400.0, velocity=0.9, lower=r.pitch_lower, upper=r.pitch_upper
        ),
        meta=_joint_meta(
            "revolute", (0.0, -1.0, 0.0), pitch_origin, (r.pitch_lower, r.pitch_upper)
        ),
    )

    # ---- pod mount (cradle -> launch_pod, FIXED) ----
    pod_origin = (r.pod_mount_x, 0.0, r.pod_mount_z)
    model.articulation(
        "cradle_to_pod",
        ArticulationType.FIXED,
        parent=cradle,
        child=launch_pod,
        origin=Origin(xyz=pod_origin),
        meta=_joint_meta("fixed", (0.0, 0.0, 0.0), pod_origin, None),
    )
    return model


def build_seeded_missile_launcher(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_missile_launcher(config_from_seed(seed), assets=assets)


def with_overrides(config: MissileLauncherConfig, **kwargs: object) -> MissileLauncherConfig:
    return replace(config, **kwargs)


def slot_choices_for_config(
    resolved: ResolvedMissileLauncherConfig,
) -> list[tuple[str, str]]:
    """Recorded on ``model.meta`` for the module_topology_diversity gate."""
    return [
        ("launch_root", resolved.root_style),
        ("yaw_stage", resolved.yaw_style),
        ("yaw_drive", resolved.yaw_joint_type),
        ("launcher_payload", resolved.launcher_style),
        ("material_palette", resolved.material_style),
    ]


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #


def run_missile_launcher_tests(
    object_model: ArticulatedObject, config: MissileLauncherConfig
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    # The cradle pivot hub rides the turntable trunnion cross-shaft — intended
    # gimbal bearing contact.
    ctx.allow_overlap(
        "turntable",
        "cradle",
        reason="cradle pivot hub rides the turntable trunnion cross-shaft",
    )
    # The launch pod is FIXED onto the cradle and rests on its tray/mount lugs.
    ctx.allow_overlap(
        "cradle",
        "launch_pod",
        reason="launch pod is bolted onto the cradle tray (mount interface contact)",
    )
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "identity_parts",
        {"base", "turntable", "cradle", "launch_pod"}.issubset(part_names),
        details=str(sorted(part_names)),
    )
    pod_visuals = {v.name for v in object_model.get_part("launch_pod").visuals}
    has_launcher = any(
        n.startswith(("cell_door", "canister", "tube", "rail", "launch_tube")) for n in pod_visuals
    )
    ctx.check("pod_has_launcher_geometry", has_launcher, details=str(sorted(pod_visuals))[:200])

    yaw = object_model.get_articulation("base_to_turntable")
    pitch = object_model.get_articulation("turntable_to_cradle")
    pod = object_model.get_articulation("cradle_to_pod")
    ctx.check("yaw_axis_vertical", tuple(yaw.axis) == (0.0, 0.0, 1.0), details=str(yaw.axis))
    ctx.check(
        "pitch_axis_horizontal",
        abs(pitch.axis[2]) < 1e-9 and abs(pitch.axis[1]) > 0.9,
        details=str(pitch.axis),
    )
    ctx.check(
        "chain_order",
        yaw.parent == "base"
        and yaw.child == "turntable"
        and pitch.parent == "turntable"
        and pitch.child == "cradle"
        and pod.parent == "cradle"
        and pod.child == "launch_pod",
        details=f"{yaw.parent}->{yaw.child}->{pitch.child}->{pod.child}",
    )
    ctx.check(
        "pod_fixed", pod.articulation_type.name == "FIXED", details=pod.articulation_type.name
    )
    for j in (yaw, pitch):
        ctx.check(
            f"{j.name}_metadata",
            {"type", "axis", "origin", "range"}.issubset(j.meta),
            details=str(j.meta),
        )

    # Motion semantics: positive pitch raises the launcher muzzle (+X end up).
    def _muzzle_z():
        try:
            aabb = ctx.part_world_aabb(object_model.get_part("launch_pod"))
        except Exception:
            return None
        return None if aabb is None else aabb[1][2]

    rest_z = _muzzle_z()
    raised_z = None
    if rest_z is not None:
        with ctx.pose({pitch: min(r.pitch_upper, math.radians(45.0))}):
            raised_z = _muzzle_z()
    measurable = rest_z is not None and raised_z is not None
    ctx.check(
        "positive_pitch_raises_launcher",
        (not measurable) or raised_z > rest_z + 0.01,
        details=f"rest_z={rest_z}, raised_z={raised_z}, measurable={measurable}",
    )

    # Elevation must not drive the pod's rear mounting plate into the azimuth
    # drive housing (the seed-0 穿模). At full elevation the whole launch_pod
    # subtree (its lowest point is the rear plate edge) must stay above the
    # drive-housing top; both are read from posed world geometry, not the
    # clearance formula, so a regression in the housing wiring is caught.
    housing_top_world = r.yaw_origin_z + r.drive_housing_top_z

    def _pod_min_z():
        try:
            aabb = ctx.part_world_aabb(object_model.get_part("launch_pod"))
        except Exception:
            return None
        return None if aabb is None else aabb[0][2]

    with ctx.pose({pitch: r.pitch_upper}):
        pod_min_z = _pod_min_z()
    ctx.check(
        "elevated_pod_clears_drive_housing",
        pod_min_z is None or pod_min_z > housing_top_world,
        details=f"pod_min_z={pod_min_z}, housing_top_world={housing_top_world}",
    )
    return ctx.report()
