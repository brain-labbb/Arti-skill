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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    clamp_joint_limits,
    mesh_from_geometry,
)

MountLayout = Literal[
    "freestanding_pole",
    "telescoping_column",
    "corner_wall_bracket",
    "mobile_trailer_mast",
    "overhead_drop",
    "side_arm_pole",
]
MastProfile = Literal["round_pole", "square_tube", "lattice", "wall_plate", "trailer_tube_stack"]
CameraStyle = Literal["bullet", "box", "dome", "ptz_pod"]
PanHeadStyle = Literal["bearing_can", "compact_socket", "yoke_bridge", "slew_bearing"]
ArmStyle = Literal["none", "side_arm", "triangular_corner_arm", "drop_rod", "braced_arm"]
PanRangeMode = Literal["limited", "continuous"]
SunshieldStyle = Literal["none", "top_plate", "three_sided", "dome_cover"]
LensStyle = Literal["flat_glass", "hooded_barrel", "dome_window"]
BraceStyle = Literal["none", "gusset", "triangular_plate", "welded_web"]
MaterialStyle = Literal["galvanized_white", "matte_black", "traffic_white", "trailer_yellow"]

PALETTES: dict[MaterialStyle, dict[str, tuple[float, float, float, float]]] = {
    "galvanized_white": {
        "metal": (0.58, 0.60, 0.62, 1.0),
        "dark": (0.10, 0.11, 0.12, 1.0),
        "camera": (0.86, 0.87, 0.86, 1.0),
        "glass": (0.16, 0.24, 0.30, 0.62),
    },
    "matte_black": {
        "metal": (0.06, 0.065, 0.07, 1.0),
        "dark": (0.015, 0.016, 0.018, 1.0),
        "camera": (0.08, 0.085, 0.09, 1.0),
        "glass": (0.05, 0.08, 0.10, 0.68),
    },
    "traffic_white": {
        "metal": (0.78, 0.80, 0.78, 1.0),
        "dark": (0.18, 0.19, 0.20, 1.0),
        "camera": (0.94, 0.94, 0.90, 1.0),
        "glass": (0.12, 0.20, 0.25, 0.62),
    },
    "trailer_yellow": {
        "metal": (0.88, 0.62, 0.12, 1.0),
        "dark": (0.12, 0.12, 0.11, 1.0),
        "camera": (0.78, 0.80, 0.78, 1.0),
        "glass": (0.10, 0.18, 0.23, 0.64),
    },
}


@dataclass(frozen=True)
class CctvMastWithPantiltCameraHeadConfig:
    mount_layout: MountLayout = "freestanding_pole"
    mast_profile: MastProfile = "round_pole"
    camera_style: CameraStyle = "bullet"
    pan_head_style: PanHeadStyle = "bearing_can"
    mast_height: float = 1.5
    telescoping_stage_count: int = 0
    extension_travel: float = 0.55
    arm_style: ArmStyle = "none"
    pan_range_mode: PanRangeMode = "limited"
    tilt_range: tuple[float, float] = (-0.6, 0.55)
    sunshield_style: SunshieldStyle = "top_plate"
    lens_style: LensStyle = "hooded_barrel"
    brace_style: BraceStyle = "gusset"
    fastener_count: int = 4
    material_style: MaterialStyle = "galvanized_white"
    name: str = "reference_cctv_mast_camera"


@dataclass(frozen=True)
class ResolvedCctvMastWithPantiltCameraHeadConfig:
    mount_layout: MountLayout
    mast_profile: MastProfile
    camera_style: CameraStyle
    pan_head_style: PanHeadStyle
    mast_height: float
    telescoping_stage_count: int
    extension_travel: float
    arm_style: ArmStyle
    pan_range_mode: PanRangeMode
    tilt_range: tuple[float, float]
    sunshield_style: SunshieldStyle
    lens_style: LensStyle
    brace_style: BraceStyle
    fastener_count: int
    material_style: MaterialStyle
    stage_widths: tuple[float, ...]
    stage_heights: tuple[float, ...]
    stage_travel: tuple[float, ...]
    pan_mount_z: float
    name: str


def config_from_seed(seed: int) -> CctvMastWithPantiltCameraHeadConfig:
    rng = random.Random(seed)
    # Keep the pole-mounted families. Telescoping trailer/wall/drop variants mix
    # mounting problems with camera-head articulation and remain excluded.
    layout: MountLayout = rng.choice(("freestanding_pole", "side_arm_pole"))
    stage_count = 0
    mast_profile: MastProfile = "round_pole"
    arm_style: ArmStyle = "side_arm" if layout == "side_arm_pole" else "none"
    return CctvMastWithPantiltCameraHeadConfig(
        mount_layout=layout,
        mast_profile=mast_profile,
        camera_style=rng.choice(("bullet", "box")),
        pan_head_style=rng.choice(("bearing_can", "compact_socket")),
        mast_height=round(rng.uniform(0.45, 4.5), 3),
        telescoping_stage_count=stage_count,
        extension_travel=0.25,
        arm_style=arm_style,
        pan_range_mode="limited",
        tilt_range=(round(rng.uniform(-1.2, -0.4), 3), round(rng.uniform(0.5, 0.9), 3)),
        sunshield_style=rng.choice(("top_plate", "three_sided")),
        lens_style=rng.choice(("flat_glass", "hooded_barrel")),
        brace_style="none",
        fastener_count=rng.choice((0, 4, 6, 8)),
        material_style=rng.choice(tuple(PALETTES)),
        name=f"seeded_cctv_mast_camera_{seed}",
    )


def resolve_config(
    config: CctvMastWithPantiltCameraHeadConfig,
) -> ResolvedCctvMastWithPantiltCameraHeadConfig:
    if config.mount_layout not in {
        "freestanding_pole",
        "telescoping_column",
        "corner_wall_bracket",
        "mobile_trailer_mast",
        "overhead_drop",
        "side_arm_pole",
    }:
        raise ValueError(f"Unsupported mount_layout: {config.mount_layout}")
    if config.mast_profile not in {
        "round_pole",
        "square_tube",
        "lattice",
        "wall_plate",
        "trailer_tube_stack",
    }:
        raise ValueError(f"Unsupported mast_profile: {config.mast_profile}")
    if config.camera_style not in {"bullet", "box", "dome", "ptz_pod"}:
        raise ValueError(f"Unsupported camera_style: {config.camera_style}")
    if config.pan_head_style not in {
        "bearing_can",
        "compact_socket",
        "yoke_bridge",
        "slew_bearing",
    }:
        raise ValueError(f"Unsupported pan_head_style: {config.pan_head_style}")
    if config.arm_style not in {
        "none",
        "side_arm",
        "triangular_corner_arm",
        "drop_rod",
        "braced_arm",
    }:
        raise ValueError(f"Unsupported arm_style: {config.arm_style}")
    if config.pan_range_mode not in {"limited", "continuous"}:
        raise ValueError(f"Unsupported pan_range_mode: {config.pan_range_mode}")
    if config.sunshield_style not in {"none", "top_plate", "three_sided", "dome_cover"}:
        raise ValueError(f"Unsupported sunshield_style: {config.sunshield_style}")
    if config.lens_style not in {"flat_glass", "hooded_barrel", "dome_window"}:
        raise ValueError(f"Unsupported lens_style: {config.lens_style}")
    if config.brace_style not in {"none", "gusset", "triangular_plate", "welded_web"}:
        raise ValueError(f"Unsupported brace_style: {config.brace_style}")
    if config.material_style not in PALETTES:
        raise ValueError(f"Unsupported material_style: {config.material_style}")
    if not 0.45 <= config.mast_height <= 4.5:
        raise ValueError("mast_height must be in [0.45, 4.5]")
    if not 0 <= config.telescoping_stage_count <= 4:
        raise ValueError("telescoping_stage_count must be in [0, 4]")
    if not 0.25 <= config.extension_travel <= 2.5:
        raise ValueError("extension_travel must be in [0.25, 2.5]")
    if config.tilt_range[0] >= config.tilt_range[1]:
        raise ValueError("tilt_range lower must be less than upper")
    if (
        config.mount_layout in {"telescoping_column", "mobile_trailer_mast"}
        and config.telescoping_stage_count < 1
    ):
        raise ValueError("telescoping layouts require telescoping_stage_count >= 1")
    if config.mount_layout == "corner_wall_bracket" and config.mast_profile != "wall_plate":
        raise ValueError("corner_wall_bracket requires mast_profile='wall_plate'")
    if config.mount_layout == "freestanding_pole" and config.mast_profile == "wall_plate":
        raise ValueError("freestanding_pole cannot use wall_plate profile")

    if config.mount_layout not in {"freestanding_pole", "side_arm_pole"}:
        config = replace(config, mount_layout="freestanding_pole")
    config = replace(
        config,
        mast_profile="round_pole",
        camera_style="bullet"
        if config.camera_style in {"dome", "ptz_pod"}
        else config.camera_style,
        pan_head_style="bearing_can"
        if config.pan_head_style in {"yoke_bridge", "slew_bearing"}
        else config.pan_head_style,
        telescoping_stage_count=0,
        extension_travel=0.25,
        arm_style="side_arm" if config.mount_layout == "side_arm_pole" else "none",
        pan_range_mode="limited",
        lens_style="hooded_barrel" if config.lens_style == "dome_window" else config.lens_style,
        brace_style="gusset" if config.mount_layout == "side_arm_pole" else "none",
    )
    sunshield_style = config.sunshield_style
    if config.camera_style == "dome":
        sunshield_style = "dome_cover"
    elif sunshield_style == "dome_cover":
        sunshield_style = "top_plate"

    stage_count = config.telescoping_stage_count
    stage_widths = tuple(0.20 * (0.82**i) for i in range(stage_count + 1))
    collapsed_height = config.mast_height / max(1, stage_count + 1)
    stage_heights = tuple(
        max(0.30, collapsed_height * (1.0 - 0.08 * i)) for i in range(stage_count + 1)
    )
    per_travel = config.extension_travel / stage_count if stage_count else 0.0
    stage_travel = tuple(per_travel for _ in range(stage_count))
    pan_mount_z = stage_heights[0] if stage_count else config.mast_height
    if config.mount_layout == "overhead_drop":
        pan_mount_z = -config.mast_height
    return ResolvedCctvMastWithPantiltCameraHeadConfig(
        mount_layout=config.mount_layout,
        mast_profile=config.mast_profile,
        camera_style=config.camera_style,
        pan_head_style=config.pan_head_style,
        mast_height=config.mast_height,
        telescoping_stage_count=stage_count,
        extension_travel=config.extension_travel,
        arm_style=config.arm_style,
        pan_range_mode=config.pan_range_mode,
        tilt_range=config.tilt_range,
        sunshield_style=sunshield_style,
        lens_style=config.lens_style,
        brace_style=config.brace_style,
        fastener_count=config.fastener_count,
        material_style=config.material_style,
        stage_widths=stage_widths,
        stage_heights=stage_heights,
        stage_travel=stage_travel,
        pan_mount_z=pan_mount_z,
        name=config.name,
    )


def _joint_meta(
    joint_type: str,
    axis: tuple[float, float, float],
    origin: tuple[float, float, float],
    joint_range: object,
) -> dict[str, object]:
    return {"type": joint_type, "axis": axis, "origin": origin, "range": joint_range}


def _pan_bearing_radius(pan_head_style: PanHeadStyle) -> float:
    return 0.065 if pan_head_style in {"bearing_can", "slew_bearing"} else 0.040


def _pan_clearance_radius(pan_head_style: PanHeadStyle) -> float:
    return 0.070 if pan_head_style == "compact_socket" else _pan_bearing_radius(pan_head_style)


# Tilt pivot in the pan_head frame — single source of truth for the tilt joint
# origin, the yoke/riser bearing geometry, and the run_tests tilt keep-out.
_TILT_PIVOT = (0.12, 0.0, 0.175)
# Camera lateral half-width (box body, the widest style); yoke arms / risers
# must stay outside this y-envelope so the camera can tilt freely between them.
_CAMERA_Y_HALF = 0.065
# Worst-case camera sweep below the tilt pivot (box camera, full down-tilt
# -1.2 rad), pivot-relative. Derived from the box body extents about the pivot
# (x 0.01..0.23, z ±0.06): the rear-bottom corner (0.01, -0.06) has radius
# 0.0608 and lands at (-0.0523, -0.0310) at full down-tilt; the front-bottom
# corner (0.23, -0.06) lands at (0.0274, -0.2361). The swept lower envelope is
# the rear-corner arc behind the pivot and the tilted bottom edge ahead of it.
# run_tests enforces that no pan_head structure inside the camera's y-slab
# pokes above this envelope (with margin).
_SWEEP_REAR_R = 0.0608
_SWEEP_REAR_CORNER = (-0.0523, -0.0310)
_SWEEP_NOSE_CORNER = (0.0274, -0.2361)


def _tilt_keepout_env(x_rel: float) -> float:
    """Lowest z (pivot-relative) the camera reaches at horizontal offset
    ``x_rel`` from the tilt pivot over the full tilt range (worst case)."""
    xr, zr = _SWEEP_REAR_CORNER
    xn, zn = _SWEEP_NOSE_CORNER
    if x_rel <= -_SWEEP_REAR_R:
        return 0.0
    if x_rel <= xr:
        return -math.sqrt(max(_SWEEP_REAR_R**2 - x_rel**2, 0.0))
    if x_rel <= xn:
        t = (x_rel - xr) / (xn - xr)
        return zr + t * (zn - zr)
    return zn


def _add_face_bolts(
    part,
    *,
    centers: tuple[tuple[float, float, float], ...],
    material,
    prefix: str,
    radius: float = 0.009,
) -> None:
    for index, center in enumerate(centers):
        part.visual(
            Cylinder(radius=radius, length=0.010),
            origin=Origin(xyz=center),
            material=material,
            name=f"{prefix}_bolt_{index}",
        )


def _add_square_tube(part, *, width: float, height: float, material, prefix: str) -> None:
    wall = max(0.008, width * 0.07)
    inner = width - 2.0 * wall
    z = height * 0.5
    part.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(width * 0.5 - wall * 0.5, 0.0, z)),
        material=material,
        name=f"{prefix}_wall_pos_x",
    )
    part.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(-width * 0.5 + wall * 0.5, 0.0, z)),
        material=material,
        name=f"{prefix}_wall_neg_x",
    )
    part.visual(
        Box((inner, wall, height)),
        origin=Origin(xyz=(0.0, width * 0.5 - wall * 0.5, z)),
        material=material,
        name=f"{prefix}_wall_pos_y",
    )
    part.visual(
        Box((inner, wall, height)),
        origin=Origin(xyz=(0.0, -width * 0.5 + wall * 0.5, z)),
        material=material,
        name=f"{prefix}_wall_neg_y",
    )


def _build_support(
    model: ArticulatedObject, resolved: ResolvedCctvMastWithPantiltCameraHeadConfig, *, metal, dark
) -> tuple[object, tuple[float, float, float]]:
    support = model.part("support")
    if resolved.mount_layout == "corner_wall_bracket":
        support.visual(
            Box((0.018, 0.28, 0.30)),
            origin=Origin(xyz=(0.0, 0.0, 0.15)),
            material=metal,
            name="wall_plate",
        )
        _add_face_bolts(
            support,
            centers=(
                (0.012, -0.105, 0.055),
                (0.012, 0.105, 0.055),
                (0.012, -0.105, 0.245),
                (0.012, 0.105, 0.245),
            ),
            material=dark,
            prefix="wall_plate",
        )
        support.visual(
            Box((0.28, 0.060, 0.045)),
            origin=Origin(xyz=(0.14, 0.0, 0.19)),
            material=metal,
            name="extension_arm",
        )
        support.visual(
            Box((0.13, 0.078, 0.018)),
            origin=Origin(xyz=(0.075, 0.0, 0.225)),
            material=dark,
            name="arm_top_clamp_plate",
        )
        support.visual(
            Box((0.16, 0.018, 0.10)),
            origin=Origin(xyz=(0.08, 0.0, 0.12), rpy=(0.0, 0.55, 0.0)),
            material=metal,
            name="triangular_gusset",
        )
        return support, (0.30, 0.0, 0.2125)
    if resolved.mount_layout == "mobile_trailer_mast":
        support.visual(
            Box((1.20, 0.62, 0.08)),
            origin=Origin(xyz=(0.0, 0.0, -0.04)),
            material=metal,
            name="trailer_deck",
        )
        support.visual(
            Box((0.45, 0.38, 0.36)),
            origin=Origin(xyz=(0.36, 0.0, 0.18)),
            material=metal,
            name="equipment_cabinet",
        )
        support.visual(
            Box((0.30, 0.30, 0.055)),
            origin=Origin(xyz=(0.0, 0.0, 0.0275)),
            material=dark,
            name="mast_pad",
        )
        for y, name in ((0.42, "left_outrigger"), (-0.42, "right_outrigger")):
            support.visual(
                Box((0.50, 0.035, 0.035)),
                origin=Origin(xyz=(0.05, y, 0.005)),
                material=metal,
                name=name,
            )
            support.visual(
                Cylinder(radius=0.030, length=0.030),
                origin=Origin(xyz=(0.28, y, -0.035)),
                material=dark,
                name=f"{name}_jack_foot",
            )
        support.visual(
            Cylinder(radius=0.12, length=0.06),
            origin=Origin(xyz=(-0.32, 0.36, -0.13), rpy=(1.5708, 0.0, 0.0)),
            material=dark,
            name="left_wheel",
        )
        support.visual(
            Cylinder(radius=0.12, length=0.06),
            origin=Origin(xyz=(-0.32, -0.36, -0.13), rpy=(1.5708, 0.0, 0.0)),
            material=dark,
            name="right_wheel",
        )
    elif resolved.mount_layout == "overhead_drop":
        support.visual(
            Cylinder(radius=0.16, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=metal,
            name="ceiling_mount_plate",
        )
        _add_face_bolts(
            support,
            centers=(
                (-0.095, -0.095, 0.014),
                (-0.095, 0.095, 0.014),
                (0.095, -0.095, 0.014),
                (0.095, 0.095, 0.014),
            ),
            material=dark,
            prefix="ceiling_plate",
            radius=0.008,
        )
        support.visual(
            Cylinder(radius=0.030, length=resolved.mast_height),
            origin=Origin(xyz=(0.0, 0.0, -resolved.mast_height * 0.5)),
            material=metal,
            name="drop_rod",
        )
        support.visual(
            Box((0.10, 0.11, 0.018)),
            origin=Origin(xyz=(0.05, 0.0, -resolved.mast_height - 0.009)),
            material=metal,
            name="hanger_socket_plate",
        )
        pan_top = 0.140 if resolved.pan_head_style == "compact_socket" else 0.095
        return support, (0.0, 0.0, -resolved.mast_height - 0.018 - pan_top)
    else:
        support.visual(
            Box((0.36, 0.36, 0.025)),
            origin=Origin(xyz=(0.0, 0.0, 0.0125)),
            material=metal,
            name="base_plate",
        )
        _add_face_bolts(
            support,
            centers=(
                (-0.135, -0.135, 0.029),
                (-0.135, 0.135, 0.029),
                (0.135, -0.135, 0.029),
                (0.135, 0.135, 0.029),
            ),
            material=dark,
            prefix="base_plate",
        )
    if resolved.telescoping_stage_count:
        return support, (0.0, 0.0, 0.025)
    if resolved.mast_profile == "square_tube":
        _add_square_tube(
            support, width=0.12, height=resolved.mast_height, material=metal, prefix="mast"
        )
    elif resolved.mast_profile == "lattice":
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                support.visual(
                    Cylinder(radius=0.012, length=resolved.mast_height),
                    origin=Origin(xyz=(sx * 0.045, sy * 0.045, resolved.mast_height * 0.5)),
                    material=metal,
                    name=f"lattice_leg_{sx}_{sy}",
                )
    else:
        if resolved.mount_layout == "side_arm_pole":
            base_r = min(0.066, resolved.mast_height * 0.028)
            tip_r = base_r * 0.68
            pole_mesh = mesh_from_geometry(
                LatheGeometry(
                    [
                        (0.0, 0.0),
                        (base_r, 0.0),
                        (tip_r, resolved.mast_height),
                        (0.0, resolved.mast_height),
                    ],
                    segments=64,
                    closed=True,
                ),
                "tapered_pole",
            )
            support.visual(
                pole_mesh,
                origin=Origin(xyz=(0.0, 0.0, 0.025)),
                material=metal,
                name="tapered_pole",
            )
        else:
            support.visual(
                Cylinder(radius=0.045, length=resolved.mast_height),
                origin=Origin(xyz=(0.0, 0.0, resolved.mast_height * 0.5)),
                material=metal,
                name="round_pole",
            )
            for z, name in (
                (0.12, "lower_pole_collar"),
                (max(0.18, resolved.mast_height - 0.10), "upper_pole_collar"),
            ):
                support.visual(
                    Cylinder(radius=0.057, length=0.030),
                    origin=Origin(xyz=(0.0, 0.0, z)),
                    material=dark,
                    name=name,
                )
    if resolved.mount_layout == "side_arm_pole":
        collar_z = resolved.mast_height + 0.020
        collar_r = min(0.066, resolved.mast_height * 0.028) * 1.24
        clamp_mesh = mesh_from_geometry(
            LatheGeometry(
                [
                    (collar_r, -0.045),
                    (collar_r + 0.030, -0.045),
                    (collar_r + 0.030, 0.045),
                    (collar_r, 0.045),
                ],
                segments=64,
                closed=True,
            ),
            "clamp_collar",
        )
        support.visual(
            clamp_mesh,
            origin=Origin(xyz=(0.0, 0.0, collar_z)),
            material=dark,
            name="clamp_collar",
        )
        # Ear/bolt offsets are DERIVED from the collar radius (which scales
        # with mast_height): the inner ear straddles the collar wall and the
        # outer ear stacks one bolt pitch outboard, so the clamp stack stays
        # one connected island for short masts too.
        collar_outer = collar_r + 0.030
        ear_ys = (-(collar_outer - 0.024), -(collar_outer - 0.024) - 0.038)
        for index, cy in enumerate(ear_ys):
            support.visual(
                Box((0.070, 0.030, 0.135)),
                origin=Origin(xyz=(0.0, cy, collar_z)),
                material=dark,
                name=f"clamp_ear_{index}",
            )
        bolt_y = 0.5 * (ear_ys[0] + ear_ys[1])
        for bz_off in (-0.045, 0.045):
            support.visual(
                Cylinder(radius=0.012, length=0.100),
                origin=Origin(xyz=(0.0, bolt_y, collar_z + bz_off), rpy=(-math.pi / 2, 0.0, 0.0)),
                material=metal,
                name=f"clamp_bolt_{abs(int(bz_off * 1000))}",
            )
        arm_length = max(0.55, resolved.mast_height * 0.36)
        arm_z = resolved.mast_height + 0.025
        support.visual(
            Cylinder(radius=0.035, length=arm_length),
            origin=Origin(xyz=(arm_length * 0.5, 0.0, arm_z), rpy=(0.0, math.pi / 2, 0.0)),
            material=metal,
            name="side_arm",
        )
        support.visual(
            Box((0.110, 0.130, 0.060)),
            origin=Origin(xyz=(0.080, 0.0, arm_z)),
            material=dark,
            name="arm_saddle",
        )
        brace_dx = arm_length * 0.38
        brace_dz = resolved.mast_height * 0.055
        brace_len = math.hypot(brace_dx, brace_dz)
        brace_pitch = math.atan2(brace_dx, brace_dz)
        for iy, by in enumerate((-0.034, 0.034)):
            support.visual(
                Cylinder(radius=0.014, length=brace_len),
                origin=Origin(
                    xyz=(brace_dx * 0.5, by, arm_z - brace_dz * 0.5),
                    rpy=(0.0, brace_pitch, 0.0),
                ),
                material=metal,
                name=f"diagonal_brace_{iy}",
            )
        socket_r = min(0.055, arm_length * 0.06)
        support.visual(
            Cylinder(radius=socket_r, length=0.120),
            origin=Origin(xyz=(arm_length, 0.0, arm_z)),
            material=dark,
            name="end_socket",
        )
        return support, (arm_length, 0.0, arm_z)
    return support, (0.0, 0.0, resolved.mast_height)


def _build_telescoping_chain(
    model: ArticulatedObject,
    support,
    resolved: ResolvedCctvMastWithPantiltCameraHeadConfig,
    *,
    metal,
    dark,
) -> tuple[object, tuple[float, float, float]]:
    parent = support
    base_z = 0.055 if resolved.mount_layout == "mobile_trailer_mast" else 0.025
    current_origin = (0.0, 0.0, base_z)
    for i in range(resolved.telescoping_stage_count + 1):
        part = model.part(f"mast_stage_{i}")
        _add_square_tube(
            part,
            width=resolved.stage_widths[i],
            height=resolved.stage_heights[i],
            material=metal,
            prefix=f"stage_{i}",
        )
        if i == 0:
            model.articulation(
                f"support_to_mast_stage_{i}",
                ArticulationType.FIXED,
                parent=parent,
                child=part,
                origin=Origin(xyz=current_origin),
            )
        else:
            origin = (0.0, 0.0, resolved.stage_heights[i - 1] * 0.62)
            model.articulation(
                f"mast_extension_{i}",
                ArticulationType.PRISMATIC,
                parent=parent,
                child=part,
                origin=Origin(xyz=origin),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(
                    effort=220.0 - i * 30.0,
                    velocity=0.20,
                    lower=0.0,
                    upper=resolved.stage_travel[i - 1],
                ),
                meta=_joint_meta(
                    "prismatic", (0.0, 0.0, 1.0), origin, (0.0, resolved.stage_travel[i - 1])
                ),
            )
            parent_width = resolved.stage_widths[i - 1]
            parent_wall = max(0.008, parent_width * 0.07)
            parent_inner = parent_width - 2.0 * parent_wall
            child_width = resolved.stage_widths[i]
            clearance = max(0.002, (parent_inner - child_width) * 0.5)
            pad_center = child_width * 0.5 + clearance * 0.5
            pad_span = child_width * 0.48
            for axis_name, sx, sy, size in (
                ("pos_x", 1.0, 0.0, (clearance, pad_span, 0.16)),
                ("neg_x", -1.0, 0.0, (clearance, pad_span, 0.16)),
                ("pos_y", 0.0, 1.0, (pad_span, clearance, 0.16)),
                ("neg_y", 0.0, -1.0, (pad_span, clearance, 0.16)),
            ):
                part.visual(
                    Box(size),
                    origin=Origin(
                        xyz=(sx * pad_center, sy * pad_center, resolved.stage_heights[i] * 0.10)
                    ),
                    material=dark,
                    name=f"stage_{i}_contact_guide_{axis_name}",
                )
        part.visual(
            Box((resolved.stage_widths[i] * 0.35, 0.010, 0.16)),
            origin=Origin(
                xyz=(0.0, resolved.stage_widths[i] * 0.5, resolved.stage_heights[i] * 0.70)
            ),
            material=dark,
            name=f"guide_pad_{i}",
        )
        if i == 0:
            part.visual(
                Box((resolved.stage_widths[i] * 1.08, 0.014, 0.055)),
                origin=Origin(xyz=(0.0, resolved.stage_widths[i] * 0.5 + 0.006, 0.12)),
                material=dark,
                name=f"stage_{i}_front_clamp_band",
            )
            part.visual(
                Box((0.014, resolved.stage_widths[i] * 1.08, 0.055)),
                origin=Origin(xyz=(resolved.stage_widths[i] * 0.5 + 0.006, 0.0, 0.12)),
                material=dark,
                name=f"stage_{i}_side_clamp_band",
            )
            for tick in range(3):
                part.visual(
                    Box((0.006, 0.004, 0.040)),
                    origin=Origin(
                        xyz=(
                            -resolved.stage_widths[i] * 0.5 - 0.004,
                            resolved.stage_widths[i] * 0.18,
                            resolved.stage_heights[i] * (0.36 + tick * 0.10),
                        )
                    ),
                    material=dark,
                    name=f"stage_{i}_height_tick_{tick}",
                )
        else:
            part.visual(
                Box((resolved.stage_widths[i] * 0.24, 0.004, 0.040)),
                origin=Origin(
                    xyz=(
                        0.0,
                        resolved.stage_widths[i] * 0.5 - 0.006,
                        resolved.stage_heights[i] * 0.62,
                    )
                ),
                material=dark,
                name=f"stage_{i}_witness_mark",
            )
        parent = part
    return parent, (0.0, 0.0, resolved.stage_heights[-1])


def _build_pan_head(
    part, resolved: ResolvedCctvMastWithPantiltCameraHeadConfig, *, metal, dark
) -> None:
    radius = _pan_bearing_radius(resolved.pan_head_style)
    part.visual(
        Cylinder(radius=radius, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=metal,
        name=resolved.pan_head_style,
    )
    part.visual(
        Cylinder(radius=radius + 0.018, length=0.012),
        # z 0.059: ring bottom 0.053 overlaps the bearing column top (0.0545)
        # so the ring/bolts stay connected to the column in every style.
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=dark,
        name="pan_bearing_top_ring",
    )
    for index, angle in enumerate((0.0, 1.5708, 3.1416, 4.7124)):
        part.visual(
            Cylinder(radius=0.006, length=0.008),
            # z 0.068: bolt bottom 0.064 embeds 1 mm into the top ring (top
            # face 0.065) so the bolts are not floating islands.
            origin=Origin(
                xyz=(radius * 0.72 * math.cos(angle), radius * 0.72 * math.sin(angle), 0.068)
            ),
            material=dark,
            name=f"pan_ring_bolt_{index}",
        )
    if resolved.pan_head_style in {"yoke_bridge", "slew_bearing", "bearing_can"}:
        # Bridge plate stays OVER the bearing can, fully behind the camera's
        # tilt sweep sector (the nose dives below pivot level at full
        # down-tilt); two lateral outriggers OUTSIDE the camera's y-envelope
        # carry the yoke arms forward to the tilt bearing.
        part.visual(
            Box((0.13, 0.20, 0.040)),
            # z 0.073: bridge bottom 0.053 embeds 1.5 mm into the bearing can
            # top (0.0545) so the can is not a floating island.
            origin=Origin(xyz=(0.0, 0.0, 0.073)),
            material=metal,
            name="yoke_bridge",
        )
        for index, y in enumerate((0.088, -0.088)):
            part.visual(
                Box((0.15, 0.020, 0.040)),
                origin=Origin(xyz=(0.065, y, 0.080)),
                material=metal,
                name=f"yoke_outrigger_{index}",
            )
        part.visual(
            Box((0.040, 0.020, 0.16)),
            origin=Origin(xyz=(_TILT_PIVOT[0], 0.088, _TILT_PIVOT[2])),
            material=metal,
            name="left_yoke_arm",
        )
        part.visual(
            Box((0.040, 0.020, 0.16)),
            origin=Origin(xyz=(_TILT_PIVOT[0], -0.088, _TILT_PIVOT[2])),
            material=metal,
            name="right_yoke_arm",
        )
        for index, y in enumerate((0.100, -0.100)):
            part.visual(
                Cylinder(radius=0.024, length=0.010),
                origin=Origin(xyz=(_TILT_PIVOT[0], y, _TILT_PIVOT[2]), rpy=(1.5708, 0.0, 0.0)),
                material=dark,
                name=f"tilt_axis_outer_cap_{index}",
            )
        part.visual(
            Box((0.030, 0.030, 0.060)),
            origin=Origin(xyz=(0.020, -0.084, 0.115)),
            material=dark,
            name="pan_cable_drop_box",
        )
    else:
        part.visual(
            Cylinder(radius=0.070, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=metal,
            name="compact_mount_flange",
        )
        # Socket block stays low and behind the tilt pivot, below the camera's
        # rear-corner sweep arc; two lateral risers OUTSIDE the camera's
        # y-envelope carry the tilt bearing (the old horizontal bridge sat
        # directly under the pivot and the camera's rear corner swept into it).
        # Block bottom (0.058) stays ABOVE the arm saddle top (0.05) so the
        # block clears the saddle for every pan angle; it stays connected to
        # the pan column through the bearing top ring (z 0.056..0.068).
        part.visual(
            Box((0.062, 0.19, 0.037)),
            origin=Origin(xyz=(0.046, 0.0, 0.0765)),
            material=dark,
            name="compact_socket_block",
        )
        for index, y in enumerate((0.0845, -0.0845)):
            part.visual(
                Box((0.050, 0.013, 0.110)),
                origin=Origin(xyz=(0.098, y, 0.120)),
                material=metal,
                name=f"compact_yoke_riser_{index}",
            )
        for index, y in enumerate((-0.093, 0.093)):
            part.visual(
                Cylinder(radius=0.018, length=0.012),
                origin=Origin(xyz=(_TILT_PIVOT[0], y, _TILT_PIVOT[2]), rpy=(1.5708, 0.0, 0.0)),
                material=dark,
                name=f"compact_tilt_socket_{index}",
            )


def _build_camera(
    part, resolved: ResolvedCctvMastWithPantiltCameraHeadConfig, *, camera_mat, dark, glass
) -> None:
    if resolved.camera_style == "box":
        part.visual(
            Box((0.22, 0.13, 0.12)),
            origin=Origin(xyz=(0.12, 0.0, 0.0)),
            material=camera_mat,
            name="camera_body",
        )
        part.visual(
            Box((0.16, 0.145, 0.012)),
            # z 0.064: lid bottom 0.058 embeds 2 mm into the body top (0.06) so
            # the lid is not a floating island.
            origin=Origin(xyz=(0.13, 0.0, 0.064)),
            material=dark,
            name="box_camera_top_service_lid",
        )
    elif resolved.camera_style == "dome":
        part.visual(
            Sphere(radius=0.055),
            origin=Origin(xyz=(0.095, 0.0, 0.018)),
            material=glass,
            name="dome_cover",
        )
        part.visual(
            Cylinder(radius=0.070, length=0.018),
            origin=Origin(xyz=(0.080, 0.0, -0.050)),
            material=camera_mat,
            name="dome_retaining_ring",
        )
        part.visual(
            Box((0.11, 0.09, 0.045)),
            origin=Origin(xyz=(0.08, 0.0, -0.030)),
            material=camera_mat,
            name="camera_core",
        )
    elif resolved.camera_style == "ptz_pod":
        part.visual(
            Cylinder(radius=0.075, length=0.12),
            origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
            material=camera_mat,
            name="ptz_pod_shell",
        )
        part.visual(
            Box((0.070, 0.105, 0.018)),
            origin=Origin(xyz=(0.055, 0.0, -0.070)),
            material=dark,
            name="ptz_lower_access_panel",
        )
    else:
        part.visual(
            Cylinder(radius=0.038, length=0.18),
            origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
            material=camera_mat,
            name="camera_barrel",
        )
        part.visual(
            Box((0.145, 0.086, 0.010)),
            # z 0.042: spine bottom 0.037 embeds 1 mm into the barrel crown
            # (top of the r=0.038 barrel) so the spine is not a floating island.
            origin=Origin(xyz=(0.105, 0.0, 0.042)),
            material=dark,
            name="bullet_camera_top_spine",
        )
    # Trunnion tips must ENGAGE the tilt bearing faces (riser inner face at
    # |y|=0.078, yoke arm inner face at |y|=0.078) — a true captured-bearing
    # overlap declared element-scoped in run_tests.
    trunnion_length = 0.162 if resolved.pan_head_style == "compact_socket" else 0.165
    part.visual(
        Cylinder(radius=0.015, length=trunnion_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.5708, 0.0, 0.0)),
        material=dark,
        name="tilt_trunnion",
    )
    for index, y in enumerate((-0.058, 0.058)):
        part.visual(
            Box((0.034, 0.012, 0.045)),
            origin=Origin(xyz=(0.012, y, 0.0)),
            material=dark,
            name=f"camera_side_mount_ear_{index}",
        )
    if resolved.lens_style == "dome_window":
        part.visual(
            Sphere(radius=0.032),
            origin=Origin(xyz=(0.205, 0.0, 0.0)),
            material=glass,
            name="front_glass",
        )
    else:
        part.visual(
            Cylinder(radius=0.030, length=0.035),
            origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
            material=dark,
            name="lens_barrel",
        )
        part.visual(
            Cylinder(radius=0.022, length=0.006),
            # x 0.224: glass rear face 0.221 embeds 1.5 mm into the lens
            # barrel (front face 0.2225) so the glass is not a floating island.
            origin=Origin(xyz=(0.224, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
            material=glass,
            name="front_glass",
        )
    part.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(-0.026, 0.0, -0.018), rpy=(0.0, 1.5708, 0.0)),
        material=dark,
        name="rear_cable_gland",
    )
    if resolved.sunshield_style in {"top_plate", "three_sided"}:
        # Shield height follows the camera crown so it rests ON the body: box
        # body top is z 0.06 (shield plate embeds into it), bullet barrel
        # crown is z 0.038 (shield plate straddles it). The rear lip hangs off
        # the shield plate so both stay one connected island.
        shield_z = 0.040 if resolved.camera_style == "bullet" else 0.055
        part.visual(
            Box((0.22, 0.12, 0.010)),
            origin=Origin(xyz=(0.12, 0.0, shield_z)),
            material=dark,
            name="sunshield_top",
        )
        part.visual(
            Box((0.035, 0.11, 0.008)),
            origin=Origin(xyz=(0.000, 0.0, shield_z - 0.008)),
            material=dark,
            name="sunshield_rear_lip",
        )
    if resolved.sunshield_style == "three_sided":
        part.visual(
            Box((0.20, 0.008, 0.040)),
            origin=Origin(xyz=(0.12, 0.060, 0.025)),
            material=dark,
            name="sunshield_side_0",
        )
        part.visual(
            Box((0.20, 0.008, 0.040)),
            origin=Origin(xyz=(0.12, -0.060, 0.025)),
            material=dark,
            name="sunshield_side_1",
        )


def build_cctv_mast_camera(
    config: CctvMastWithPantiltCameraHeadConfig | None = None, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    config = config or CctvMastWithPantiltCameraHeadConfig()
    resolved = resolve_config(config)
    # Leave assets=None as-is: compile/clamp then resolves meshes through the
    # active asset session (where mesh_from_geometry writes them). Forcing a
    # fresh AssetContext here would point mesh refs at an empty directory.
    model = ArticulatedObject(name=resolved.name, assets=assets)
    model.meta["adopted_source_ids"] = ("S1", "S2", "S3", "S4")
    palette = PALETTES[resolved.material_style]
    metal = model.material("mast_galvanized", rgba=palette["metal"])
    dark = model.material("mast_dark_hardware", rgba=palette["dark"])
    camera_mat = model.material("camera_body_finish", rgba=palette["camera"])
    glass = model.material("lens_glass", rgba=palette["glass"])
    support, pan_origin = _build_support(model, resolved, metal=metal, dark=dark)
    pan_parent = support
    if resolved.telescoping_stage_count:
        pan_parent, local_pan_origin = _build_telescoping_chain(
            model, support, resolved, metal=metal, dark=dark
        )
        pan_origin = local_pan_origin
    if resolved.arm_style in {
        "side_arm",
        "braced_arm",
        "triangular_corner_arm",
        "drop_rod",
    } and resolved.mount_layout not in {"corner_wall_bracket", "overhead_drop"}:
        arm = model.part("side_arm")
        if resolved.arm_style == "drop_rod":
            arm.visual(
                Cylinder(radius=0.024, length=0.28),
                origin=Origin(xyz=(0.0, 0.0, -0.14)),
                material=metal,
                name="drop_rod",
            )
            next_pan_origin = (0.0, 0.0, -0.28)
        else:
            if resolved.telescoping_stage_count:
                arm_start_x = resolved.stage_widths[-1] * 0.5
            elif resolved.mount_layout == "side_arm_pole":
                arm_start_x = 0.0
            else:
                arm_start_x = 0.045
            arm_length = 0.28
            arm_height = 0.040
            arm.visual(
                Box((arm_length, 0.055, arm_height)),
                origin=Origin(xyz=(arm_start_x + arm_length * 0.5, 0.0, arm_height * 0.5)),
                material=metal,
                name=resolved.arm_style,
            )
            # Single source of truth for the pan bearing seat. ``pan_axis_x`` is
            # BOTH the pan joint origin AND the outboard reach of the arm's
            # mounting saddle, so the vertical pan axis always passes through
            # real arm hardware (no phantom mid-air pivot). The saddle overlaps
            # only the coaxial pan bearing cylinder — a rotation-invariant seat
            # overlap declared via ``allow_overlap`` in run_tests.
            arm_end_x = arm_start_x + arm_length
            pan_axis_x = arm_end_x + _pan_clearance_radius(resolved.pan_head_style)
            saddle_x0 = arm_end_x - 0.030
            saddle_x1 = pan_axis_x + 0.020
            # Saddle top at arm_height + 0.004: proud of the arm so the pan
            # axis lands on real hardware, but low enough that the rotating
            # yoke bridge (bottom z 0.053 in the pan frame) keeps >= 8 mm
            # clearance over it for every pan angle.
            arm.visual(
                Box((saddle_x1 - saddle_x0, 0.055, 0.020)),
                origin=Origin(xyz=(0.5 * (saddle_x0 + saddle_x1), 0.0, arm_height - 0.006)),
                material=metal,
                name="pan_mount_saddle",
            )
            next_pan_origin = (pan_axis_x, 0.0, 0.0)
        if resolved.brace_style != "none" and resolved.arm_style != "drop_rod":
            # Brace must clear BOTH neighbours: its tip stays outside the pan
            # head's swept annulus (max pan-head corner radius hypot(0.144,
            # 0.105) ~= 0.178 about the axis at arm_start_x + arm_length +
            # clearance), and its inboard end stays clear of the arm-root end
            # socket (r 0.033). Extent of this pitched box is center +/-
            # (0.055*cos(0.40) + 0.020*sin(0.40)) ~= +/- 0.058, so center
            # arm_start_x + 0.095 spans 0.037..0.153: 4 mm off the socket and
            # >= 8 mm of radial clearance to the sweep for every style.
            arm.visual(
                Box((0.11, 0.014, 0.040)),
                origin=Origin(xyz=(arm_start_x + 0.095, 0.0, 0.070), rpy=(0.0, 0.40, 0.0)),
                material=metal,
                name=resolved.brace_style,
            )
        model.articulation(
            "support_to_arm",
            ArticulationType.FIXED,
            parent=pan_parent,
            child=arm,
            origin=Origin(xyz=pan_origin),
        )
        pan_parent = arm
        pan_origin = next_pan_origin
    pan_head = model.part("pan_head")
    _build_pan_head(pan_head, resolved, metal=metal, dark=dark)
    camera = model.part("camera_head")
    _build_camera(camera, resolved, camera_mat=camera_mat, dark=dark, glass=glass)
    if resolved.pan_range_mode == "continuous":
        model.articulation(
            "pan_joint",
            ArticulationType.CONTINUOUS,
            parent=pan_parent,
            child=pan_head,
            origin=Origin(xyz=pan_origin),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=35.0, velocity=1.4),
            meta=_joint_meta("continuous", (0.0, 0.0, 1.0), pan_origin, "unbounded"),
        )
    else:
        model.articulation(
            "pan_joint",
            ArticulationType.REVOLUTE,
            parent=pan_parent,
            child=pan_head,
            origin=Origin(xyz=pan_origin),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-2.6, upper=2.6),
            meta=_joint_meta("revolute", (0.0, 0.0, 1.0), pan_origin, (-2.6, 2.6)),
        )
    tilt_origin = _TILT_PIVOT
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=tilt_origin),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0, velocity=1.0, lower=resolved.tilt_range[0], upper=resolved.tilt_range[1]
        ),
        meta=_joint_meta("revolute", (0.0, -1.0, 0.0), tilt_origin, resolved.tilt_range),
    )
    if getattr(pan_parent, "name", None) == "side_arm" and resolved.pan_range_mode != "continuous":
        # Arm-mounted heads: at full down-tilt the camera nose dips below
        # arm-top level, so panning it back over the arm shank would collide.
        # The arm cannot move (it must run under the head), so derive the true
        # pan travel against the arm/mast geometry at the worst-case tilt.
        # Only the coaxial bearing seat is exempted, element-scoped.
        arm_elem = resolved.arm_style
        seat_pairs = [
            # Coaxial pan bearing seated on the saddle / abutting the arm tip:
            # axisymmetric about the pan axis, rotation-invariant contacts.
            ("side_arm", "pan_head", "pan_mount_saddle", resolved.pan_head_style),
            ("side_arm", "pan_head", arm_elem, resolved.pan_head_style),
            ("side_arm", "pan_head", arm_elem, "compact_mount_flange"),
            ("side_arm", "pan_head", "pan_mount_saddle", "compact_mount_flange"),
        ]
        new_limits = clamp_joint_limits(
            model,
            "pan_joint",
            margin=0.008,
            keepout=["side_arm", "support"],
            allowed_pairs=seat_pairs,
            base_pose={"tilt_joint": resolved.tilt_range[0]},
        )
        model.get_articulation("pan_joint").meta["range"] = (new_limits.lower, new_limits.upper)
    return model


def build_seeded_cctv_mast_camera(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_cctv_mast_camera(config_from_seed(seed), assets=assets)


def with_overrides(
    config: CctvMastWithPantiltCameraHeadConfig, **kwargs: object
) -> CctvMastWithPantiltCameraHeadConfig:
    return replace(config, **kwargs)


def run_cctv_mast_camera_tests(
    object_model: ArticulatedObject, config: CctvMastWithPantiltCameraHeadConfig
) -> TestReport:
    resolved = resolve_config(config)
    ctx = TestContext(object_model)
    part_names = {part.name for part in object_model.parts}
    ctx.check(
        "identity_parts",
        {"support", "pan_head", "camera_head"}.issubset(part_names),
        details=str(part_names),
    )
    camera_visuals = {v.name for v in object_model.get_part("camera_head").visuals}
    ctx.check("camera_has_lens", "front_glass" in camera_visuals, details=str(camera_visuals))
    pan = object_model.get_articulation("pan_joint")
    tilt = object_model.get_articulation("tilt_joint")
    ctx.check("pan_axis_vertical", tuple(pan.axis) == (0.0, 0.0, 1.0), details=str(pan.axis))
    ctx.check(
        "tilt_axis_horizontal",
        abs(tilt.axis[2]) < 1e-6 and (abs(tilt.axis[0]) > 0.9 or abs(tilt.axis[1]) > 0.9),
        details=str(tilt.axis),
    )
    ctx.check(
        "camera_child_of_pan_head",
        tilt.parent == "pan_head" and tilt.child == "camera_head",
        details=f"{tilt.parent}->{tilt.child}",
    )
    ctx.check(
        "pan_metadata",
        {"type", "axis", "origin", "range"}.issubset(pan.meta),
        details=str(pan.meta),
    )
    # Anti-phantom guard: the vertical pan axis must pass through real hardware
    # of the pan joint's parent link (arm saddle) — not float in mid-air off the
    # arm tip. Check the origin's (x, y) lands inside a parent Box footprint.
    pan_parent_name = pan.parent
    ox, oy, _oz = (float(pan.origin.xyz[0]), float(pan.origin.xyz[1]), float(pan.origin.xyz[2]))
    if pan_parent_name in part_names:
        parent_part = object_model.get_part(pan_parent_name)
        axis_on_hw = False
        for v in parent_part.visuals:
            geom = v.geometry
            size = getattr(geom, "size", None)
            if size is None or len(size) != 3:
                continue
            cx, cy = float(v.origin.xyz[0]), float(v.origin.xyz[1])
            hx, hy = float(size[0]) * 0.5 + 0.015, float(size[1]) * 0.5 + 0.015
            if abs(ox - cx) <= hx and abs(oy - cy) <= hy:
                axis_on_hw = True
                break
        ctx.check(
            "pan_axis_on_parent_hardware",
            axis_on_hw,
            details=f"pan origin=({ox:.3f},{oy:.3f}) parent={pan_parent_name!r}",
        )
        # Seat overlap between the arm's mounting saddle and the coaxial pan
        # bearing is a legitimate rotation-invariant mount; declare it so the
        # harness overlap gate does not flag it.
        if pan_parent_name == "side_arm" and any(
            v.name == "pan_mount_saddle" for v in parent_part.visuals
        ):
            ctx.allow_overlap(
                "side_arm",
                "pan_head",
                reason="pan bearing seats on the arm mounting saddle (coaxial, rotation-invariant)",
                elem_a="pan_mount_saddle",
                elem_b=resolved.pan_head_style,
            )
    ctx.check(
        "tilt_metadata",
        {"type", "axis", "origin", "range"}.issubset(tilt.meta),
        details=str(tilt.meta),
    )
    # Camera tilt keep-out: no pan_head structure inside the camera's y-slab
    # may poke above the camera's worst-case swept lower envelope (full
    # down-tilt of the box body — the widest/deepest camera style, so this is
    # conservative for bullet/dome/ptz_pod). This is the guard that keeps the
    # yoke bridge / socket block out of the tilt arc for good.
    px, _py, pz = _TILT_PIVOT
    slab = _CAMERA_Y_HALF + 0.005
    keepout_violations: list[str] = []
    for v in object_model.get_part("pan_head").visuals:
        geom = v.geometry
        vx, vy, vz = (float(v.origin.xyz[0]), float(v.origin.xyz[1]), float(v.origin.xyz[2]))
        size = getattr(geom, "size", None)
        if size is not None and len(size) == 3:
            x_half, y_half, z_half = (float(size[0]) / 2, float(size[1]) / 2, float(size[2]) / 2)
        else:
            radius = float(getattr(geom, "radius", 0.0) or 0.0)
            length = float(getattr(geom, "length", 0.0) or 0.0)
            x_half, y_half, z_half = radius, radius, length / 2
        if abs(vy) - y_half >= slab:
            continue  # laterally clear of the camera for every tilt angle
        z_top_rel = (vz + z_half) - pz
        allowed_top = _tilt_keepout_env((vx + x_half) - px) - 0.008
        if z_top_rel > allowed_top:
            keepout_violations.append(
                f"{v.name}: z_top_rel={z_top_rel:.4f} > allowed {allowed_top:.4f}"
            )
    ctx.check(
        "pan_head_clear_of_camera_tilt_sweep",
        not keepout_violations,
        details="; ".join(keepout_violations),
    )
    # True captured tilt-bearing overlaps, element-scoped: the trunnion axle
    # (coaxial with the tilt axis, rotation-invariant) engages the yoke arms /
    # risers that carry the bearing. Nothing else on camera<->pan_head is
    # allowed, so a camera-body-into-mount 穿模 stays catchable.
    if resolved.pan_head_style == "compact_socket":
        bearing_elems = ("compact_yoke_riser_0", "compact_yoke_riser_1")
    else:
        bearing_elems = ("left_yoke_arm", "right_yoke_arm")
    for eb in bearing_elems:
        ctx.allow_overlap(
            "camera_head",
            "pan_head",
            elem_a="tilt_trunnion",
            elem_b=eb,
            reason="tilt axle captured in the yoke bearing (coaxial with the tilt axis)",
        )
    # The arm shank seats inside the support arm's end socket (FIXED socket
    # capture at the weld interface).
    if pan_parent_name == "side_arm" and "support" in part_names:
        support_part = object_model.get_part("support")
        if any(v.name == "end_socket" for v in support_part.visuals):
            ctx.allow_overlap(
                "support",
                "side_arm",
                elem_a="end_socket",
                elem_b=resolved.arm_style,
                reason="arm shank seats in the support arm's end socket (fixed socket capture)",
            )
    if resolved.telescoping_stage_count:
        extension_joints = [
            j for j in object_model.articulations if j.name.startswith("mast_extension_")
        ]
        ctx.check(
            "telescoping_joint_count",
            len(extension_joints) == resolved.telescoping_stage_count,
            details=str([j.name for j in extension_joints]),
        )
        ctx.check(
            "telescoping_axis",
            all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in extension_joints),
            details=str([j.axis for j in extension_joints]),
        )
    return ctx.report()
