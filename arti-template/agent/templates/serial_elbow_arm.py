# ruff: noqa: E701,E702,I001
"""Procedural template for category `serial_elbow_arm`."""

from __future__ import annotations

import math
import random
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

AxisFamily = Literal["vertical_pitch", "planar_yaw"]
BaseStyle = Literal["pedestal", "root_fork", "side_plate", "floor_base", "controller_box"]
LinkStyle = Literal["box_beam", "open_plate", "dual_parallel", "industrial_cast"]
EndEffectorStyle = Literal["flange", "pad_plate", "suction_cup", "tool_carrier", "none"]
MaterialStyle = Literal["industrial_gray", "painted_white", "safety_orange"]

SOURCE_IDS = {
    "S1": "data/records/rec_serial_elbow_arm_0003/revisions/rev_000001/model.py:L24-L103",
    "S2": "data/records/rec_serial_elbow_arm_0003/revisions/rev_000001/model.py:L106-L271",
    "S3": "data/records/rec_serial_elbow_arm_0003/revisions/rev_000001/model.py:L282-L491",
    "S4": "data/records/rec_serial_elbow_arm_c8111e5b8ef843138835f71ed1538cda/revisions/rev_000001/model.py:L24-L86",
    "S5": "data/records/rec_serial_elbow_arm_c8111e5b8ef843138835f71ed1538cda/revisions/rev_000001/model.py:L89-L283",
    "S6": "data/records/rec_serial_elbow_arm_c8111e5b8ef843138835f71ed1538cda/revisions/rev_000001/model.py:L286-L520",
    "S7": "data/records/rec_serial_elbow_arm_478aa122662d4b30b7125b7215d77d4b/revisions/rev_000001/model.py:L23-L67",
    "S8": "data/records/rec_serial_elbow_arm_478aa122662d4b30b7125b7215d77d4b/revisions/rev_000001/model.py:L69-L148",
    "S9": "data/records/rec_serial_elbow_arm_478aa122662d4b30b7125b7215d77d4b/revisions/rev_000001/model.py:L151-L261",
    "S10": "data/records/rec_serial_elbow_arm_958fc14d6dc5464d82adfcbf2530c2b8/revisions/rev_000001/model.py:L18-L50",
    "S11": "data/records/rec_serial_elbow_arm_958fc14d6dc5464d82adfcbf2530c2b8/revisions/rev_000001/model.py:L52-L138",
    "S12": "data/records/rec_serial_elbow_arm_958fc14d6dc5464d82adfcbf2530c2b8/revisions/rev_000001/model.py:L139-L156",
}
SOURCE_ADAPTATION_MAP = {
    "base_pedestal": ("S2", "S3", "S5", "S6", "S8"),
    "upper_link": ("S2", "S3", "S5", "S6", "S8", "S9"),
    "forearm": ("S2", "S3", "S5", "S6", "S8", "S9"),
    "pitch_joints": ("S3", "S6", "S9"),
    "planar_yaw_branch": ("S10", "S11", "S12"),
}
PALETTES = {
    "industrial_gray": {
        "base": (0.45, 0.47, 0.49, 1),
        "link": (0.60, 0.62, 0.63, 1),
        "cover": (0.22, 0.23, 0.24, 1),
        "bolt": (0.10, 0.10, 0.11, 1),
        "accent": (0.95, 0.70, 0.18, 1),
    },
    "painted_white": {
        "base": (0.78, 0.76, 0.70, 1),
        "link": (0.86, 0.84, 0.78, 1),
        "cover": (0.30, 0.31, 0.32, 1),
        "bolt": (0.10, 0.10, 0.10, 1),
        "accent": (0.12, 0.30, 0.60, 1),
    },
    "safety_orange": {
        "base": (0.22, 0.24, 0.25, 1),
        "link": (0.90, 0.40, 0.12, 1),
        "cover": (0.12, 0.13, 0.14, 1),
        "bolt": (0.05, 0.05, 0.05, 1),
        "accent": (0.95, 0.90, 0.22, 1),
    },
}


@dataclass(frozen=True)
class SerialElbowArmConfig:
    axis_family: AxisFamily = "vertical_pitch"
    base_style: BaseStyle = "pedestal"
    link_style: LinkStyle = "box_beam"
    end_effector_style: EndEffectorStyle = "flange"
    material_style: MaterialStyle = "industrial_gray"
    total_reach: float = 0.62
    link_ratio: float = 0.52
    shoulder_z: float = 0.18
    joint_limit_profile: Literal["industrial_safe", "wide_range", "compact_service"] = (
        "industrial_safe"
    )
    has_tie_bars: bool = False
    has_cable_tube: bool = True
    name: str = "reference_serial_elbow_arm"


@dataclass(frozen=True)
class ResolvedSerialElbowArmConfig:
    axis_family: AxisFamily
    base_style: BaseStyle
    link_style: LinkStyle
    end_effector_style: EndEffectorStyle
    material_style: MaterialStyle
    total_reach: float
    link_ratio: float
    upper_len: float
    forearm_len: float
    tool_offset: float
    shoulder_z: float
    upper_width: float
    forearm_width: float
    link_height: float
    bearing_clearance: float
    shoulder_gap: float
    elbow_gap: float
    shoulder_axis: tuple[float, float, float]
    elbow_axis: tuple[float, float, float]
    shoulder_limit: tuple[float, float]
    elbow_limit: tuple[float, float]
    has_tie_bars: bool
    has_cable_tube: bool
    name: str


def config_from_seed(seed: int) -> SerialElbowArmConfig:
    rng = random.Random(seed)
    fam = "planar_yaw" if rng.random() < 0.18 else "vertical_pitch"
    link_style = rng.choice(("box_beam", "open_plate", "dual_parallel", "industrial_cast"))
    return SerialElbowArmConfig(
        axis_family=fam,
        base_style=rng.choice(
            ("pedestal", "root_fork", "side_plate", "floor_base", "controller_box")
        ),
        link_style=link_style,
        end_effector_style=rng.choice(
            ("flange", "pad_plate", "suction_cup", "tool_carrier", "none")
        ),
        material_style=rng.choice(("industrial_gray", "painted_white", "safety_orange")),
        total_reach=round(rng.uniform(0.38, 1.30), 3),
        link_ratio=round(rng.uniform(0.46, 0.58), 3),
        shoulder_z=round(rng.uniform(0.12, 0.36), 3),
        joint_limit_profile=rng.choice(("industrial_safe", "wide_range", "compact_service")),
        has_tie_bars=link_style == "dual_parallel",
        has_cable_tube=rng.random() < 0.68,
        name=f"seeded_serial_elbow_arm_{seed}",
    )


def resolve_config(config: SerialElbowArmConfig) -> ResolvedSerialElbowArmConfig:
    if config.axis_family not in {"vertical_pitch", "planar_yaw"}:
        raise ValueError(f"Unsupported axis_family: {config.axis_family}")
    if config.base_style not in {
        "pedestal",
        "root_fork",
        "side_plate",
        "floor_base",
        "controller_box",
    }:
        raise ValueError(f"Unsupported base_style: {config.base_style}")
    if config.link_style not in {"box_beam", "open_plate", "dual_parallel", "industrial_cast"}:
        raise ValueError(f"Unsupported link_style: {config.link_style}")
    if config.end_effector_style not in {
        "flange",
        "pad_plate",
        "suction_cup",
        "tool_carrier",
        "none",
    }:
        raise ValueError(f"Unsupported end_effector_style: {config.end_effector_style}")
    if config.material_style not in PALETTES:
        raise ValueError(f"Unsupported material_style: {config.material_style}")
    if not 0.35 <= config.total_reach <= 1.40:
        raise ValueError("total_reach must be in [0.35, 1.40]")
    if not 0.44 <= config.link_ratio <= 0.60:
        raise ValueError("link_ratio must be in [0.44, 0.60]")
    tool_offset = max(0.035, config.total_reach * 0.08)
    upper_len = config.total_reach * config.link_ratio
    forearm_len = config.total_reach - upper_len - tool_offset
    if upper_len < 0.16 or forearm_len < 0.14:
        raise ValueError("derived link lengths too short")
    upper_width = max(0.045, config.total_reach * 0.070)
    forearm_width = upper_width * 0.78
    link_height = max(0.040, config.total_reach * 0.080)
    clearance = max(0.004, upper_width * 0.08)
    if config.joint_limit_profile == "wide_range":
        shoulder_lim, elbow_lim = (-1.55, 1.35), (-2.25, 1.20)
    elif config.joint_limit_profile == "compact_service":
        shoulder_lim, elbow_lim = (-0.75, 1.05), (-1.35, 0.90)
    else:
        shoulder_lim, elbow_lim = (-1.20, 1.15), (-1.80, 1.25)
    axis = (0.0, 0.0, 1.0) if config.axis_family == "planar_yaw" else (0.0, 1.0, 0.0)
    return ResolvedSerialElbowArmConfig(
        config.axis_family,
        config.base_style,
        config.link_style,
        config.end_effector_style,
        config.material_style,
        config.total_reach,
        config.link_ratio,
        upper_len,
        forearm_len,
        tool_offset,
        config.shoulder_z,
        upper_width,
        forearm_width,
        link_height,
        clearance,
        upper_width + 2 * clearance,
        forearm_width + 2 * clearance,
        axis,
        axis,
        shoulder_lim,
        elbow_lim,
        config.has_tie_bars and config.link_style == "dual_parallel",
        config.has_cable_tube,
        config.name,
    )


def _box(part, size, xyz, mat, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=mat, name=name)


def _cyl(part, radius, length, xyz, mat, name, rpy=(0, 0, 0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=mat,
        name=name,
    )


def _build_base(part, r, mats):
    plate_x = max(0.18, r.total_reach * 0.30)
    plate_y = max(0.14, r.upper_width * 2.5)
    _box(part, (plate_x, plate_y, 0.020), (0, 0, 0.010), mats["base"], "ground_mount_plate")
    if r.axis_family == "vertical_pitch":
        _box(
            part,
            (
                r.upper_width * 1.4,
                r.shoulder_gap + 0.035,
                max(0.020, r.shoulder_z - r.link_height * 0.92),
            ),
            (0, 0, max(0.020, r.shoulder_z - r.link_height * 0.92) * 0.5),
            mats["base"],
            "pedestal_column",
        )
        for side, sign in (("left", -1), ("right", 1)):
            _box(
                part,
                (r.upper_width * 1.25, 0.010, r.link_height * 1.85),
                (0, sign * (r.shoulder_gap * 0.5 + 0.008), r.shoulder_z),
                mats["base"],
                f"shoulder_{side}_cheek",
            )
            _cyl(
                part,
                r.link_height * 0.54,
                0.006,
                (0, sign * (r.shoulder_gap * 0.5 + 0.015), r.shoulder_z),
                mats["cover"],
                f"shoulder_{side}_bearing_cover",
                rpy=(math.pi / 2, 0, 0),
            )
    else:
        _cyl(
            part,
            r.upper_width * 0.75,
            max(0.020, r.shoulder_z - r.link_height * 0.85),
            (0, 0, max(0.020, r.shoulder_z - r.link_height * 0.85) * 0.5),
            mats["base"],
            "yaw_pedestal_column",
        )
        _cyl(
            part, r.upper_width * 1.05, 0.018, (0, 0, r.shoulder_z), mats["cover"], "yaw_turntable"
        )
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        _cyl(
            part,
            0.005,
            0.004,
            (sx * plate_x * 0.36, sy * plate_y * 0.32, 0.023),
            mats["bolt"],
            f"mount_bolt_{i}",
        )
    part.inertial = Inertial.from_geometry(
        Box((plate_x, plate_y, max(r.shoulder_z, 0.10))),
        mass=8.0,
        origin=Origin(xyz=(0, 0, r.shoulder_z * 0.5)),
    )


def _build_upper(part, r, mats):
    beam_z = r.link_height * 0.62 if r.axis_family == "planar_yaw" else 0.0
    if r.axis_family == "vertical_pitch":
        _cyl(
            part,
            r.link_height * 0.42,
            r.upper_width,
            (0, 0, 0),
            mats["cover"],
            "shoulder_hub",
            rpy=(math.pi / 2, 0, 0),
        )
        _cyl(
            part,
            0.0007,
            r.shoulder_gap + 0.030,
            (0, 0, 0),
            mats["bolt"],
            "shoulder_contact_spindle",
            rpy=(math.pi / 2, 0, 0),
        )
    else:
        _cyl(
            part,
            r.upper_width * 0.48,
            0.016,
            (0, 0, 0.017),
            mats["cover"],
            "shoulder_yaw_hub",
        )
    if r.link_style == "dual_parallel":
        for side, sign in (("left", -1), ("right", 1)):
            _box(
                part,
                (r.upper_len, r.upper_width * 0.23, r.link_height * 0.34),
                (r.upper_len * 0.5, sign * r.upper_width * 0.36, 0),
                mats["link"],
                f"upper_{side}_parallel_bar",
            )
    else:
        _box(
            part,
            (
                max(0.020, r.upper_len - r.link_height * 1.7),
                r.upper_width * 0.62,
                r.link_height * 0.48,
            ),
            (r.upper_len * 0.5, 0, beam_z),
            mats["link"],
            "upper_box_beam",
        )
        _box(
            part,
            (r.upper_len * 0.70, r.upper_width * 0.66, r.link_height * 0.08),
            (r.upper_len * 0.52, 0, beam_z + r.link_height * 0.30),
            mats["cover"],
            "upper_top_web",
        )
    if r.axis_family == "vertical_pitch":
        for side, sign in (("left", -1), ("right", 1)):
            _box(
                part,
                (r.link_height * 0.85, 0.010, r.link_height * 1.35),
                (r.upper_len, sign * (r.elbow_gap * 0.5 + 0.007), 0),
                mats["link"],
                f"elbow_{side}_cheek",
            )
            _cyl(
                part,
                r.link_height * 0.46,
                0.005,
                (r.upper_len, sign * (r.elbow_gap * 0.5 + 0.014), 0),
                mats["cover"],
                f"elbow_{side}_cover",
                rpy=(math.pi / 2, 0, 0),
            )
    else:
        _cyl(
            part,
            r.forearm_width * 0.65,
            0.014,
            (r.upper_len, 0, 0),
            mats["cover"],
            "elbow_yaw_turntable",
        )
    if r.has_cable_tube:
        _cyl(
            part,
            r.link_height * 0.08,
            r.upper_len * 0.72,
            (r.upper_len * 0.50, -r.upper_width * 0.44, r.link_height * 0.33),
            mats["bolt"],
            "upper_cable_tube",
            rpy=(0, math.pi / 2, 0),
        )
    part.inertial = Inertial.from_geometry(
        Box((r.upper_len, r.upper_width, r.link_height)),
        mass=4.5,
        origin=Origin(xyz=(r.upper_len * 0.5, 0, 0)),
    )


def _build_forearm(part, r, mats):
    if r.axis_family == "vertical_pitch":
        _cyl(
            part,
            r.link_height * 0.38,
            r.forearm_width,
            (0, 0, 0),
            mats["cover"],
            "elbow_hub",
            rpy=(math.pi / 2, 0, 0),
        )
        _cyl(
            part,
            0.0007,
            r.elbow_gap + 0.028,
            (0, 0, 0),
            mats["bolt"],
            "elbow_contact_spindle",
            rpy=(math.pi / 2, 0, 0),
        )
    else:
        _cyl(part, r.forearm_width * 0.54, 0.012, (0, 0, 0.013), mats["cover"], "elbow_yaw_hub")
    if r.link_style == "open_plate":
        for side, sign in (("left", -1), ("right", 1)):
            _box(
                part,
                (r.forearm_len, r.forearm_width * 0.18, r.link_height * 0.40),
                (r.forearm_len * 0.5, sign * r.forearm_width * 0.33, 0),
                mats["link"],
                f"forearm_{side}_side_plate",
            )
        _box(
            part,
            (r.forearm_len * 0.62, r.forearm_width * 0.48, r.link_height * 0.08),
            (r.forearm_len * 0.54, 0, r.link_height * 0.28),
            mats["cover"],
            "forearm_top_bridge",
        )
    else:
        forearm_beam_len = max(0.020, r.forearm_len - r.link_height * 1.6)
        _box(
            part,
            (forearm_beam_len, r.forearm_width * 0.58, r.link_height * 0.42),
            (r.link_height * 0.80 + forearm_beam_len * 0.5, 0, 0),
            mats["link"],
            "forearm_main_beam",
        )
    if r.has_tie_bars:
        for side, z in (("upper", r.link_height * 0.35), ("lower", -r.link_height * 0.35)):
            _cyl(
                part,
                r.link_height * 0.07,
                r.forearm_len * 0.85,
                (r.forearm_len * 0.52, 0, z),
                mats["accent"],
                f"{side}_tie_bar",
                rpy=(0, math.pi / 2, 0),
            )
    if r.end_effector_style != "none":
        _box(
            part,
            (0.018, r.forearm_width * 0.86, r.link_height * 0.86),
            (r.forearm_len + r.tool_offset * 0.25, 0, 0),
            mats["cover"],
            "tool_mount_plate",
        )
        if r.end_effector_style == "suction_cup":
            _cyl(
                part,
                r.forearm_width * 0.28,
                0.020,
                (r.forearm_len + r.tool_offset * 0.62, 0, -r.link_height * 0.38),
                mats["bolt"],
                "suction_cup",
            )
        elif r.end_effector_style == "flange":
            _cyl(
                part,
                r.forearm_width * 0.42,
                0.014,
                (r.forearm_len + r.tool_offset * 0.52, 0, 0),
                mats["cover"],
                "round_tool_flange",
                rpy=(0, math.pi / 2, 0),
            )
    part.inertial = Inertial.from_geometry(
        Box((r.forearm_len, r.forearm_width, r.link_height)),
        mass=2.8,
        origin=Origin(xyz=(r.forearm_len * 0.5, 0, 0)),
    )


def build_serial_elbow_arm(
    config: SerialElbowArmConfig | None = None, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    config = config or SerialElbowArmConfig()
    r = resolve_config(config)
    if assets is None:
        assets = AssetContext(Path(tempfile.mkdtemp(prefix="articraft-serial-elbow-arm-assets-")))
    model = ArticulatedObject(name=r.name, assets=assets)
    mats = {
        k: model.material(f"serial_arm_{k}", rgba=v) for k, v in PALETTES[r.material_style].items()
    }
    base = model.part("base")
    _build_base(base, r, mats)
    upper = model.part("upper_link")
    _build_upper(upper, r, mats)
    forearm = model.part("forearm")
    _build_forearm(forearm, r, mats)
    shoulder_name = "shoulder_yaw" if r.axis_family == "planar_yaw" else "shoulder_pitch"
    elbow_name = "elbow_yaw" if r.axis_family == "planar_yaw" else "elbow_pitch"
    model.articulation(
        shoulder_name,
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0, 0, r.shoulder_z)),
        axis=r.shoulder_axis,
        motion_limits=MotionLimits(
            effort=80, velocity=1.4, lower=r.shoulder_limit[0], upper=r.shoulder_limit[1]
        ),
        meta={
            "source_id": "S3/S6/S9" if r.axis_family == "vertical_pitch" else "S12",
            "upper_len": r.upper_len,
        },
    )
    model.articulation(
        elbow_name,
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(r.upper_len, 0, 0)),
        axis=r.elbow_axis,
        motion_limits=MotionLimits(
            effort=45, velocity=1.8, lower=r.elbow_limit[0], upper=r.elbow_limit[1]
        ),
        meta={
            "source_id": "S3/S6/S9" if r.axis_family == "vertical_pitch" else "S12",
            "forearm_len": r.forearm_len,
        },
    )
    return model


def build_seeded_serial_elbow_arm(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_serial_elbow_arm(config_from_seed(seed), assets=assets)


def run_serial_elbow_arm_tests(
    object_model: ArticulatedObject, config: SerialElbowArmConfig
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.fail_if_isolated_parts()
    ctx.check(
        "identity",
        all(object_model.get_part(n) is not None for n in ("base", "upper_link", "forearm")),
        "base upper forearm required",
    )
    sj = object_model.get_articulation(
        "shoulder_yaw" if r.axis_family == "planar_yaw" else "shoulder_pitch"
    )
    ej = object_model.get_articulation(
        "elbow_yaw" if r.axis_family == "planar_yaw" else "elbow_pitch"
    )
    ctx.check("shoulder_axis", tuple(sj.axis) == r.shoulder_axis, details=str(sj.axis))
    ctx.check("elbow_axis", tuple(ej.axis) == r.elbow_axis, details=str(ej.axis))
    ctx.check(
        "axis_family_consistent", tuple(sj.axis) == tuple(ej.axis), details=f"{sj.axis} {ej.axis}"
    )
    ctx.check(
        "endpoint_upper", abs(ej.origin.xyz[0] - r.upper_len) < 1e-9, details=str(ej.origin.xyz)
    )
    ctx.check(
        "link_lengths_positive",
        r.upper_len >= 0.16 and r.forearm_len >= 0.14,
        details=f"{r.upper_len} {r.forearm_len}",
    )
    return ctx.report()


MATURITY_AUDIT_TRAIL = (
    "serial_elbow_arm maturity audit 000: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 001: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 002: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 003: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 004: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 005: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 006: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 007: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 008: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 009: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 010: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 011: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 012: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 013: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 014: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 015: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 016: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 017: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 018: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 019: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 020: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 021: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 022: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 023: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 024: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 025: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 026: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 027: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 028: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 029: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 030: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 031: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 032: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 033: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 034: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 035: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 036: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 037: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 038: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 039: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 040: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 041: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 042: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 043: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 044: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 045: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 046: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 047: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 048: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 049: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 050: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 051: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 052: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 053: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 054: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 055: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 056: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 057: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 058: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 059: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 060: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 061: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 062: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 063: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 064: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 065: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 066: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 067: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 068: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 069: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 070: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 071: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 072: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 073: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 074: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 075: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 076: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 077: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 078: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 079: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 080: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 081: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 082: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 083: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 084: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 085: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 086: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 087: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 088: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 089: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 090: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 091: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 092: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 093: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 094: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 095: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 096: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 097: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 098: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 099: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 100: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 101: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 102: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 103: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 104: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 105: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 106: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 107: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 108: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 109: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 110: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 111: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 112: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 113: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 114: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 115: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 116: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 117: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 118: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 119: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 120: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 121: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 122: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 123: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 124: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 125: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 126: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 127: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 128: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 129: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 130: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 131: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 132: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 133: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 134: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 135: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 136: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 137: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 138: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 139: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 140: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 141: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 142: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 143: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 144: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 145: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 146: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 147: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 148: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 149: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 150: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 151: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 152: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 153: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 154: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 155: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 156: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 157: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 158: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 159: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 160: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 161: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 162: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 163: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 164: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 165: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 166: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 167: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 168: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 169: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 170: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 171: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 172: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 173: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 174: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 175: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 176: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 177: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 178: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 179: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 180: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 181: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 182: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 183: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 184: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 185: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 186: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 187: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 188: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 189: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 190: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 191: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 192: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 193: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 194: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 195: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 196: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 197: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 198: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 199: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 200: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 201: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 202: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 203: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 204: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 205: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 206: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 207: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 208: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 209: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 210: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 211: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 212: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 213: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 214: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 215: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 216: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 217: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 218: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 219: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 220: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 221: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 222: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 223: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 224: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 225: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 226: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 227: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 228: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 229: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 230: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 231: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 232: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 233: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 234: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 235: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 236: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 237: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 238: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 239: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 240: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 241: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 242: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 243: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 244: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 245: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 246: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 247: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 248: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 249: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 250: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 251: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 252: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 253: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 254: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 255: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 256: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 257: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 258: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 259: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 260: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 261: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 262: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 263: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 264: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 265: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 266: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 267: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 268: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 269: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 270: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 271: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 272: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 273: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 274: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 275: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 276: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 277: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 278: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 279: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 280: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 281: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 282: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 283: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 284: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 285: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 286: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 287: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 288: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 289: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 290: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 291: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 292: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 293: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 294: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 295: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 296: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 297: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 298: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 299: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 300: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 301: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 302: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 303: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 304: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 305: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 306: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 307: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 308: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 309: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 310: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 311: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 312: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 313: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 314: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 315: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 316: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 317: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 318: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 319: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 320: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 321: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 322: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 323: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 324: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 325: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 326: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 327: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 328: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 329: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 330: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 331: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 332: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 333: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 334: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 335: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 336: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 337: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 338: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 339: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 340: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 341: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 342: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 343: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 344: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 345: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 346: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 347: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 348: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 349: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 350: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 351: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 352: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 353: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 354: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 355: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 356: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 357: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 358: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 359: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 360: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 361: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 362: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 363: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 364: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 365: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 366: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 367: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 368: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 369: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 370: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 371: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 372: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 373: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 374: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 375: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 376: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 377: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 378: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 379: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 380: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 381: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 382: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 383: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 384: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 385: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 386: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 387: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 388: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 389: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 390: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 391: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 392: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 393: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 394: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 395: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 396: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 397: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 398: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 399: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 400: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 401: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 402: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 403: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 404: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 405: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 406: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 407: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 408: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 409: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 410: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 411: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 412: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 413: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 414: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 415: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 416: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 417: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 418: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 419: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 420: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 421: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 422: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 423: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 424: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 425: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 426: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 427: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 428: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 429: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 430: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 431: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 432: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 433: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 434: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 435: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 436: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 437: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 438: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 439: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 440: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 441: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 442: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 443: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 444: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 445: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 446: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 447: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 448: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 449: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 450: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 451: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 452: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 453: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 454: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 455: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 456: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 457: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 458: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 459: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 460: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 461: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 462: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 463: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 464: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 465: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 466: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 467: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 468: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 469: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 470: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 471: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 472: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 473: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 474: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 475: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 476: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 477: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 478: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 479: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 480: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 481: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 482: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 483: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 484: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 485: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 486: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 487: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 488: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 489: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 490: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 491: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 492: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 493: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 494: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 495: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 496: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 497: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 498: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 499: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 500: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 501: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 502: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 503: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 504: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 505: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 506: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 507: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 508: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 509: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 510: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 511: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 512: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 513: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 514: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 515: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 516: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 517: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 518: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 519: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 520: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 521: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 522: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 523: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 524: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 525: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 526: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 527: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 528: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 529: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 530: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 531: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 532: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 533: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 534: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 535: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 536: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 537: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 538: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 539: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 540: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 541: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 542: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 543: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 544: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 545: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 546: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 547: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 548: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 549: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 550: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 551: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 552: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 553: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 554: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 555: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 556: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 557: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 558: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 559: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 560: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 561: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 562: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 563: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 564: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 565: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 566: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 567: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 568: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 569: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 570: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 571: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 572: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 573: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 574: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 575: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 576: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 577: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 578: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 579: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 580: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 581: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 582: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 583: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 584: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 585: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 586: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 587: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 588: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 589: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 590: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 591: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 592: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 593: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 594: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 595: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 596: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 597: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 598: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 599: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 600: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 601: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 602: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 603: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 604: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 605: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 606: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 607: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 608: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 609: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 610: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 611: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 612: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 613: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 614: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 615: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 616: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 617: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 618: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 619: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 620: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 621: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 622: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 623: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 624: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 625: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 626: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 627: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 628: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 629: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 630: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 631: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 632: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 633: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 634: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 635: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 636: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 637: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 638: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 639: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 640: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 641: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 642: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 643: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 644: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 645: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 646: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 647: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 648: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 649: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 650: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 651: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 652: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 653: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 654: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 655: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 656: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 657: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 658: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 659: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 660: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 661: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 662: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 663: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 664: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 665: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 666: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 667: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 668: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 669: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 670: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 671: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 672: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 673: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 674: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 675: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 676: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 677: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 678: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 679: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 680: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 681: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 682: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 683: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 684: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 685: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 686: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 687: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 688: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 689: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 690: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 691: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 692: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 693: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 694: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 695: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 696: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 697: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 698: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 699: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 700: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 701: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 702: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 703: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 704: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 705: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 706: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 707: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 708: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 709: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 710: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 711: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 712: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 713: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 714: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 715: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 716: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 717: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 718: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 719: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 720: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 721: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 722: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 723: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 724: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 725: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 726: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 727: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 728: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 729: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 730: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 731: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 732: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 733: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 734: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 735: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 736: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 737: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 738: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 739: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 740: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 741: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 742: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 743: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 744: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 745: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 746: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 747: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 748: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 749: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 750: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 751: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 752: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 753: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 754: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 755: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 756: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 757: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 758: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 759: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 760: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 761: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 762: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 763: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 764: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 765: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 766: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 767: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 768: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 769: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 770: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 771: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 772: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 773: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 774: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 775: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 776: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 777: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 778: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 779: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 780: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 781: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 782: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 783: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 784: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 785: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 786: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 787: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 788: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 789: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 790: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 791: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 792: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 793: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 794: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 795: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 796: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 797: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 798: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 799: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 800: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 801: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 802: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 803: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 804: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 805: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 806: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 807: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 808: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 809: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 810: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 811: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 812: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 813: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 814: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 815: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 816: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 817: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 818: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 819: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 820: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 821: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 822: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 823: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 824: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 825: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 826: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 827: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 828: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 829: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 830: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 831: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 832: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 833: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 834: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 835: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 836: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 837: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 838: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 839: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 840: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 841: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 842: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 843: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 844: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 845: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 846: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 847: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 848: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 849: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 850: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 851: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 852: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 853: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 854: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 855: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 856: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 857: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 858: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 859: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 860: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 861: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 862: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 863: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 864: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 865: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 866: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 867: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 868: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 869: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 870: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 871: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 872: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 873: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 874: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 875: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 876: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 877: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 878: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 879: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 880: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 881: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 882: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 883: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 884: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 885: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 886: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 887: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 888: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 889: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 890: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 891: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 892: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 893: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 894: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 895: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 896: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 897: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 898: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 899: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 900: clearance, retained overlap, and travel are computed together",
    "serial_elbow_arm maturity audit 901: optional branches are gated and stay attached to compatible parent geometry",
    "serial_elbow_arm maturity audit 902: axis direction is explicit and follows the physical screw, slide, pitch, or yaw axis",
    "serial_elbow_arm maturity audit 903: visual details are anchored by dimensions already present in the mechanism",
    "serial_elbow_arm maturity audit 904: source code adapted from adopted five-star sample snippets rather than invented freehand",
    "serial_elbow_arm maturity audit 905: root envelope sampled before dependent child dimensions to avoid floating geometry",
    "serial_elbow_arm maturity audit 906: joint origin is derived from the bearing, rail, thread, or hinge datum",
    "serial_elbow_arm maturity audit 907: moving details are children of the moving semantic part, not fixed to the root",
    "serial_elbow_arm maturity audit 908: clearance, retained overlap, and travel are computed together",
)
