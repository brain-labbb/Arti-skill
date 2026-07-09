"""Procedural template for category `stand_mixer`.

Coordinate convention (model-local frame):

- +X is the FRONT of the mixer (where the bowl/tool live).
- -X is the REAR (where the rear_column rises and the head pivots).
- +Y is RIGHT, -Y is LEFT.
- +Z is UP. The mixer_base footprint sits on z=0.

Spine: mixer_base envelope (W along Y, D along X, H along Z) is the primary
driver. Everything else (rear_column height, mixer_head dimensions, bowl
diameter, tool drop, control placement) is derived from the envelope.
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
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    section_loft,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Discrete style knobs (spec section 10).
# ---------------------------------------------------------------------------

HeadShape = Literal["rounded", "boxy", "retro", "industrial"]
BaseShape = Literal["oval", "rectangle", "rounded"]
BowlShape = Literal["deep_bowl", "wide_bowl", "tapered"]
BowlSizeClass = Literal["compact", "standard", "large"]
ToolType = Literal["dough_hook", "whisk", "flat_beater"]
BowlLiftType = Literal["none", "slide_horizontal", "lever_lift"]
BowlSlideClass = Literal["short", "medium"]
SpeedSelectorStyle = Literal["knob", "lever", "dial", "none"]
SpeedSelectorMount = Literal["base_fixed", "head_mounted"]
MaterialStyle = Literal["pastel", "stainless", "black", "retro"]
DetailLevel = Literal["minimal", "normal", "detailed"]
SlideAxis = Literal["x", "y"]

# ---------------------------------------------------------------------------
# Discrete bucket + continuous range (Section 0 requirement).
# bowl_slide_range is the spec's required bucket+continuous knob.
# ---------------------------------------------------------------------------

BOWL_SLIDE_RANGE_M: dict[BowlSlideClass, tuple[float, float]] = {
    "short": (0.025, 0.050),
    "medium": (0.055, 0.090),
}

# Discrete bucket + per-bucket continuous range for bowl size, per spec
# section 0 ("离散桶 + 连续范围"). Each bucket multiplies the shape's base
# (outer_r, depth) so two seeds with the same bowl_shape can still differ.
BOWL_SIZE_SCALE: dict[BowlSizeClass, tuple[float, float]] = {
    "compact": (0.84, 0.93),
    "standard": (0.96, 1.06),
    "large": (1.10, 1.22),
}

# ---------------------------------------------------------------------------
# Material palettes.
# ---------------------------------------------------------------------------

MATERIAL_PALETTES: dict[MaterialStyle, dict[str, tuple[float, float, float, float]]] = {
    "pastel": {
        "body": (0.92, 0.82, 0.78, 1.0),
        "trim": (0.96, 0.96, 0.94, 1.0),
        "head": (0.92, 0.82, 0.78, 1.0),
        "bowl": (0.88, 0.88, 0.90, 1.0),
        "tool": (0.80, 0.82, 0.84, 1.0),
        "hardware": (0.55, 0.55, 0.58, 1.0),
        "knob": (0.18, 0.18, 0.20, 1.0),
        "rubber": (0.10, 0.10, 0.10, 1.0),
    },
    "stainless": {
        "body": (0.80, 0.82, 0.84, 1.0),
        "trim": (0.92, 0.92, 0.92, 1.0),
        "head": (0.80, 0.82, 0.84, 1.0),
        "bowl": (0.88, 0.90, 0.92, 1.0),
        "tool": (0.86, 0.88, 0.90, 1.0),
        "hardware": (0.42, 0.43, 0.45, 1.0),
        "knob": (0.20, 0.20, 0.22, 1.0),
        "rubber": (0.08, 0.08, 0.08, 1.0),
    },
    "black": {
        "body": (0.16, 0.16, 0.17, 1.0),
        "trim": (0.36, 0.36, 0.37, 1.0),
        "head": (0.16, 0.16, 0.17, 1.0),
        "bowl": (0.86, 0.86, 0.88, 1.0),
        "tool": (0.80, 0.82, 0.84, 1.0),
        "hardware": (0.55, 0.55, 0.58, 1.0),
        "knob": (0.60, 0.55, 0.18, 1.0),
        "rubber": (0.08, 0.08, 0.08, 1.0),
    },
    "retro": {
        "body": (0.90, 0.76, 0.30, 1.0),
        "trim": (0.96, 0.92, 0.74, 1.0),
        "head": (0.90, 0.76, 0.30, 1.0),
        "bowl": (0.86, 0.86, 0.88, 1.0),
        "tool": (0.80, 0.82, 0.84, 1.0),
        "hardware": (0.50, 0.36, 0.18, 1.0),
        "knob": (0.30, 0.18, 0.10, 1.0),
        "rubber": (0.18, 0.10, 0.06, 1.0),
    },
}


# ---------------------------------------------------------------------------
# Geometry constants. All in meters. Stand mixer is a tabletop appliance so the
# whole envelope is on the order of ~0.25 m wide and ~0.45 m tall.
# ---------------------------------------------------------------------------

BASE_WIDTH_Y = 0.230  # Y span (left-right)
BASE_DEPTH_X = 0.380  # X span (front-back). Rear column anchors near -X end.
BASE_HEIGHT_Z = 0.105  # base body height above z=0

COLUMN_HEIGHT_Z = 0.300  # rear_column height above top of base (must allow tallest bowl)
COLUMN_WIDTH_Y = 0.130
COLUMN_DEPTH_X = 0.140

HEAD_LENGTH_X = 0.300  # head extends from column toward +X
HEAD_WIDTH_Y = 0.140
HEAD_HEIGHT_Z = 0.130

TOOL_ATTACHMENT_DROP = 0.130  # length of tool hanging below head
BOWL_DEPTH_Z = 0.170
BOWL_OUTER_RADIUS = 0.115
BOWL_INNER_RADIUS = 0.103

CARRIAGE_HEIGHT_Z = 0.018
WALL = 0.012

# Margin between the rear column +X face and the head pivot axis.
HEAD_PIVOT_X_FROM_COLUMN_FACE = 0.006


# ---------------------------------------------------------------------------
# Config dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StandMixerConfig:
    head_shape: HeadShape = "rounded"
    base_shape: BaseShape = "rounded"
    bowl_shape: BowlShape = "deep_bowl"
    bowl_size_class: BowlSizeClass = "standard"
    bowl_size_scale: float | None = None
    tool_type: ToolType = "flat_beater"
    head_tilt_range_deg: float = 45.0
    bowl_lift_type: BowlLiftType = "none"
    bowl_slide_class: BowlSlideClass = "medium"
    bowl_slide_range: float | None = None
    bowl_slide_axis: SlideAxis = "x"
    speed_selector_style: SpeedSelectorStyle = "knob"
    speed_selector_mount: SpeedSelectorMount = "base_fixed"
    control_count: int = 1
    has_head_lock: bool = False
    material_style: MaterialStyle = "pastel"
    detail_level: DetailLevel = "normal"
    name: str = "reference_stand_mixer"


@dataclass(frozen=True)
class ResolvedStandMixerConfig:
    head_shape: HeadShape
    base_shape: BaseShape
    bowl_shape: BowlShape
    bowl_size_class: BowlSizeClass
    bowl_size_scale: float
    tool_type: ToolType
    head_tilt_range_rad: float
    bowl_lift_type: BowlLiftType
    bowl_slide_class: BowlSlideClass
    bowl_slide_range: float
    bowl_slide_axis: SlideAxis
    lever_lift_travel: float
    speed_selector_style: SpeedSelectorStyle
    speed_selector_mount: SpeedSelectorMount
    control_count: int
    has_head_lock: bool
    material_style: MaterialStyle
    detail_level: DetailLevel
    # Spine envelope (derived).
    base_w: float
    base_d: float
    base_h: float
    column_h: float
    head_len: float
    head_w: float
    head_h: float
    bowl_outer_r: float
    bowl_inner_r: float
    bowl_depth: float
    bowl_center_x: float  # rest position of bowl center, world X
    head_pivot_x: float  # world X of the head_tilt joint axis
    head_pivot_z: float  # world Z of the head_tilt joint axis
    tool_drop_z: float  # how far the tool hangs below the head bottom
    name: str


# ---------------------------------------------------------------------------
# Seed -> config
# ---------------------------------------------------------------------------


def config_from_seed(seed: int) -> StandMixerConfig:
    rng = random.Random(seed)
    head_shape = rng.choice(("rounded", "boxy", "retro", "industrial"))
    base_shape = rng.choice(("oval", "rectangle", "rounded"))
    bowl_shape = rng.choice(("deep_bowl", "wide_bowl", "tapered"))
    bowl_size_class: BowlSizeClass = rng.choices(
        ("compact", "standard", "large"), weights=(0.30, 0.45, 0.25), k=1
    )[0]
    size_lo, size_hi = BOWL_SIZE_SCALE[bowl_size_class]
    bowl_size_scale = round(rng.uniform(size_lo, size_hi), 4)
    tool_type = rng.choice(("dough_hook", "whisk", "flat_beater"))
    head_tilt_range_deg = round(rng.uniform(25.0, 60.0), 1)

    # Important: ensure all three bowl_lift_type values appear across seeds.
    bowl_lift_type = rng.choices(
        ("none", "slide_horizontal", "lever_lift"),
        weights=(0.40, 0.35, 0.25),
        k=1,
    )[0]
    bowl_slide_class: BowlSlideClass = rng.choice(("short", "medium"))
    bowl_slide_lo, bowl_slide_hi = BOWL_SLIDE_RANGE_M[bowl_slide_class]
    bowl_slide_range = round(rng.uniform(bowl_slide_lo, bowl_slide_hi), 4)
    bowl_slide_axis: SlideAxis = rng.choice(("x", "y"))

    speed_selector_style = rng.choices(
        ("knob", "lever", "dial", "none"), weights=(0.40, 0.20, 0.25, 0.15), k=1
    )[0]
    speed_selector_mount = rng.choices(("base_fixed", "head_mounted"), weights=(0.65, 0.35), k=1)[0]
    control_count = rng.choices((1, 2, 3), weights=(0.55, 0.30, 0.15), k=1)[0]
    if speed_selector_style == "none":
        control_count = 0
    has_head_lock = rng.random() < 0.35

    material_style = rng.choice(("pastel", "stainless", "black", "retro"))
    detail_level = rng.choice(("minimal", "normal", "detailed"))

    return StandMixerConfig(
        head_shape=head_shape,
        base_shape=base_shape,
        bowl_shape=bowl_shape,
        bowl_size_class=bowl_size_class,
        bowl_size_scale=bowl_size_scale,
        tool_type=tool_type,
        head_tilt_range_deg=head_tilt_range_deg,
        bowl_lift_type=bowl_lift_type,
        bowl_slide_class=bowl_slide_class,
        bowl_slide_range=bowl_slide_range,
        bowl_slide_axis=bowl_slide_axis,
        speed_selector_style=speed_selector_style,
        speed_selector_mount=speed_selector_mount,
        control_count=control_count,
        has_head_lock=has_head_lock,
        material_style=material_style,
        detail_level=detail_level,
        name=f"seeded_stand_mixer_{seed}",
    )


# ---------------------------------------------------------------------------
# Resolution: derive spine geometry from envelope + style knobs.
# ---------------------------------------------------------------------------


def resolve_config(config: StandMixerConfig) -> ResolvedStandMixerConfig:
    if config.head_shape not in ("rounded", "boxy", "retro", "industrial"):
        raise ValueError(f"Unsupported head_shape: {config.head_shape}")
    if config.base_shape not in ("oval", "rectangle", "rounded"):
        raise ValueError(f"Unsupported base_shape: {config.base_shape}")
    if config.bowl_shape not in ("deep_bowl", "wide_bowl", "tapered"):
        raise ValueError(f"Unsupported bowl_shape: {config.bowl_shape}")
    if config.bowl_size_class not in BOWL_SIZE_SCALE:
        raise ValueError(f"Unsupported bowl_size_class: {config.bowl_size_class}")
    if config.tool_type not in ("dough_hook", "whisk", "flat_beater"):
        raise ValueError(f"Unsupported tool_type: {config.tool_type}")
    if not (24.0 <= config.head_tilt_range_deg <= 61.0):
        raise ValueError(
            f"head_tilt_range_deg must be in [25, 60]; got {config.head_tilt_range_deg}"
        )
    if config.bowl_lift_type not in ("none", "slide_horizontal", "lever_lift"):
        raise ValueError(f"Unsupported bowl_lift_type: {config.bowl_lift_type}")
    if config.bowl_slide_class not in BOWL_SLIDE_RANGE_M:
        raise ValueError(f"Unsupported bowl_slide_class: {config.bowl_slide_class}")
    if config.bowl_slide_axis not in ("x", "y"):
        raise ValueError(f"Unsupported bowl_slide_axis: {config.bowl_slide_axis}")
    if config.speed_selector_style not in ("knob", "lever", "dial", "none"):
        raise ValueError(f"Unsupported speed_selector_style: {config.speed_selector_style}")
    if config.speed_selector_mount not in ("base_fixed", "head_mounted"):
        raise ValueError(f"Unsupported speed_selector_mount: {config.speed_selector_mount}")
    if config.speed_selector_style == "none":
        if config.control_count != 0:
            raise ValueError("control_count must be 0 when speed_selector_style is none")
    else:
        if config.control_count not in (1, 2, 3):
            raise ValueError("control_count must be 1, 2, or 3 when controls exist")
    if config.material_style not in MATERIAL_PALETTES:
        raise ValueError(f"Unsupported material_style: {config.material_style}")
    if config.detail_level not in ("minimal", "normal", "detailed"):
        raise ValueError(f"Unsupported detail_level: {config.detail_level}")

    # Continuous bucket-derived slide range.
    if config.bowl_slide_range is None:
        bowl_slide_range = sum(BOWL_SLIDE_RANGE_M[config.bowl_slide_class]) / 2.0
    else:
        bowl_slide_range = config.bowl_slide_range
    bowl_slide_range = max(0.020, min(bowl_slide_range, 0.095))

    head_tilt_range_rad = math.radians(config.head_tilt_range_deg)

    # Per-shape base proportions (kept inside the shape's design space).
    if config.bowl_shape == "wide_bowl":
        shape_outer = BOWL_OUTER_RADIUS * 1.08
        shape_inner = BOWL_INNER_RADIUS * 1.08
        shape_depth = BOWL_DEPTH_Z * 0.70
    elif config.bowl_shape == "tapered":
        shape_outer = BOWL_OUTER_RADIUS * 0.95
        shape_inner = BOWL_INNER_RADIUS * 0.95
        shape_depth = BOWL_DEPTH_Z * 0.85
    else:  # deep_bowl
        shape_outer = BOWL_OUTER_RADIUS
        shape_inner = BOWL_INNER_RADIUS
        shape_depth = BOWL_DEPTH_Z * 0.90

    # Discrete bucket + per-bucket continuous scale (size_class + size_scale)
    # multiplies the shape's base, so two seeds with the same bowl_shape can
    # still differ in absolute size.
    if config.bowl_size_scale is None:
        size_lo, size_hi = BOWL_SIZE_SCALE[config.bowl_size_class]
        bowl_size_scale = (size_lo + size_hi) / 2.0
    else:
        bowl_size_scale = config.bowl_size_scale
    bowl_size_scale = max(0.80, min(bowl_size_scale, 1.25))
    bowl_outer_r = shape_outer * bowl_size_scale
    bowl_inner_r = shape_inner * bowl_size_scale
    bowl_depth = shape_depth * bowl_size_scale

    # Geometric self-consistency (spec section 0, line 17): the bowl top must
    # fit under the head at rest. Compute headroom = column_h minus the head's
    # downward extent + a safety margin, then clamp bowl_depth (and the
    # corresponding outer_r so the bowl scales uniformly).
    head_drop_below_pivot = HEAD_HEIGHT_Z * 0.55 + 0.020  # socket sits this far below pivot
    safety = 0.012
    max_bowl_depth = COLUMN_HEIGHT_Z - head_drop_below_pivot - safety
    if bowl_depth > max_bowl_depth:
        effective_scale = max_bowl_depth / shape_depth
        bowl_size_scale = min(bowl_size_scale, effective_scale)
        bowl_outer_r = shape_outer * bowl_size_scale
        bowl_inner_r = shape_inner * bowl_size_scale
        bowl_depth = shape_depth * bowl_size_scale

    # Base envelope.
    base_w = BASE_WIDTH_Y
    base_d = BASE_DEPTH_X
    base_h = BASE_HEIGHT_Z
    column_h = COLUMN_HEIGHT_Z
    head_len = HEAD_LENGTH_X
    head_w = HEAD_WIDTH_Y
    head_h = HEAD_HEIGHT_Z

    # Head pivot lives at the top of the rear column, on its forward face.
    column_front_face_x = -base_d / 2.0 + COLUMN_DEPTH_X
    head_pivot_x = column_front_face_x + HEAD_PIVOT_X_FROM_COLUMN_FACE
    head_pivot_z = base_h + column_h

    # The tool socket sits under the head, far from the pivot (toward the
    # front of the head). Place the bowl center under the tool axis.
    head_socket_x_local = head_len * 0.75  # along head's local +X from pivot
    bowl_center_x = head_pivot_x + head_socket_x_local

    # Lever lift travel chosen to clearly lower the bowl by a few cm.
    lever_lift_travel = 0.040

    return ResolvedStandMixerConfig(
        head_shape=config.head_shape,
        base_shape=config.base_shape,
        bowl_shape=config.bowl_shape,
        bowl_size_class=config.bowl_size_class,
        bowl_size_scale=bowl_size_scale,
        tool_type=config.tool_type,
        head_tilt_range_rad=head_tilt_range_rad,
        bowl_lift_type=config.bowl_lift_type,
        bowl_slide_class=config.bowl_slide_class,
        bowl_slide_range=bowl_slide_range,
        bowl_slide_axis=config.bowl_slide_axis,
        lever_lift_travel=lever_lift_travel,
        speed_selector_style=config.speed_selector_style,
        speed_selector_mount=config.speed_selector_mount,
        control_count=config.control_count,
        has_head_lock=config.has_head_lock,
        material_style=config.material_style,
        detail_level=config.detail_level,
        base_w=base_w,
        base_d=base_d,
        base_h=base_h,
        column_h=column_h,
        head_len=head_len,
        head_w=head_w,
        head_h=head_h,
        bowl_outer_r=bowl_outer_r,
        bowl_inner_r=bowl_inner_r,
        bowl_depth=bowl_depth,
        bowl_center_x=bowl_center_x,
        head_pivot_x=head_pivot_x,
        head_pivot_z=head_pivot_z,
        tool_drop_z=TOOL_ATTACHMENT_DROP,
        name=config.name,
    )


# ---------------------------------------------------------------------------
# Builders for the major sub-shapes. The mixer_base contains the base body,
# the rear_column, motor-housing details on the head are kept on the head
# part. All non-articulating decorations are attached via `parent.visual(...)`.
# ---------------------------------------------------------------------------


def _build_base_body(
    base,
    resolved: ResolvedStandMixerConfig,
    *,
    assets: AssetContext,
    body_mat,
    trim_mat,
) -> None:
    """Build the mixer_base shell as a stack of visuals on the base part.

    Geometry origin convention: base_body is centered along Y at y=0, the
    base body spans x in [-base_d/2, +base_d/2] along X with the rear at
    -base_d/2 (where the column rises).
    """
    W = resolved.base_w
    D = resolved.base_d
    H = resolved.base_h

    base_mesh = mesh_from_geometry(
        _build_base_loft_geometry(resolved.base_shape, D, W, H),
        assets.mesh_path(f"mixer_base_shell_{resolved.base_shape}.obj"),
    )
    base.visual(
        base_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=body_mat,
        name="base_main_slab",
    )

    # Decorative front trim plate
    base.visual(
        Box((D * 0.6, 0.005, H * 0.5)),
        origin=Origin(xyz=(D * 0.18, W / 2.0 - 0.004, H * 0.5)),
        material=trim_mat,
        name="base_trim_side_r",
    )
    base.visual(
        Box((D * 0.6, 0.005, H * 0.5)),
        origin=Origin(xyz=(D * 0.18, -W / 2.0 + 0.004, H * 0.5)),
        material=trim_mat,
        name="base_trim_side_l",
    )

    # Four small rubber feet under the base (visual only).
    foot_radius = max(0.010, min(W, D) * 0.045)
    foot_height = 0.010
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            base.visual(
                Cylinder(radius=foot_radius, length=foot_height),
                origin=Origin(
                    xyz=(
                        sx * (D / 2.0 - foot_radius - 0.018),
                        sy * (W / 2.0 - foot_radius - 0.018),
                        foot_height / 2.0,
                    ),
                ),
                material=trim_mat,
                name=f"foot_{'p' if sx > 0 else 'm'}{'p' if sy > 0 else 'm'}",
            )


def _build_rear_column(base, resolved: ResolvedStandMixerConfig, *, body_mat, trim_mat) -> None:
    """Attach the rear_column as a visual on the base part. The column rises
    from the rear of the base envelope and provides the pivot for the head.
    """
    D = resolved.base_d
    H = resolved.base_h
    col_h = resolved.column_h
    col_w = COLUMN_WIDTH_Y
    col_d = COLUMN_DEPTH_X
    # Column centered at x = -D/2 + col_d/2; column_top_z = H + col_h.
    base.visual(
        Box((col_d, col_w, col_h)),
        origin=Origin(xyz=(-D / 2.0 + col_d / 2.0, 0.0, H + col_h / 2.0)),
        material=body_mat,
        name="rear_column",
    )
    # Decorative cylindrical cap on top of the column where the head pivots.
    cap_r = col_w * 0.45
    base.visual(
        Cylinder(radius=cap_r, length=col_w * 0.96),
        origin=Origin(
            xyz=(resolved.head_pivot_x - HEAD_PIVOT_X_FROM_COLUMN_FACE, 0.0, H + col_h),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_mat,
        name="rear_column_pivot_cap",
    )

    if resolved.detail_level in ("normal", "detailed"):
        # Decorative bezel ring at base of column.
        base.visual(
            Box((col_d * 1.08, col_w * 1.04, 0.012)),
            origin=Origin(xyz=(-D / 2.0 + col_d / 2.0, 0.0, H + 0.006)),
            material=trim_mat,
            name="rear_column_skirt",
        )
    if resolved.detail_level == "detailed":
        # Vertical chrome accent strip on the front face of the column.
        base.visual(
            Box((0.005, col_w * 0.6, col_h * 0.85)),
            origin=Origin(
                xyz=(-D / 2.0 + col_d - 0.0025, 0.0, H + col_h * 0.50),
            ),
            material=trim_mat,
            name="rear_column_accent",
        )


def _add_speed_selector_on_base(
    base,
    resolved: ResolvedStandMixerConfig,
    *,
    hardware_mat,
    knob_mat,
) -> None:
    """Add base-mounted controls as visual decorations on the base part."""
    if resolved.speed_selector_style == "none" or resolved.control_count == 0:
        return
    D = resolved.base_d
    H = resolved.base_h
    W = resolved.base_w
    n = resolved.control_count
    style = resolved.speed_selector_style

    # Controls sit on the front face (+X side) of the base, along the upper
    # part of the front face.
    front_x = D / 2.0 - 0.006
    z_center = H * 0.62
    span = W * 0.55
    if n == 1:
        ys = (0.0,)
    elif n == 2:
        ys = (-span / 3.0, span / 3.0)
    else:
        ys = (-span / 2.5, 0.0, span / 2.5)

    for i, y in enumerate(ys):
        if style == "knob":
            base.visual(
                Cylinder(radius=0.018, length=0.020),
                origin=Origin(
                    xyz=(front_x + 0.004, y, z_center),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=knob_mat,
                name=f"speed_selector_knob_{i}",
            )
            base.visual(
                Cylinder(radius=0.024, length=0.006),
                origin=Origin(
                    xyz=(front_x - 0.001, y, z_center),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=hardware_mat,
                name=f"speed_selector_bezel_{i}",
            )
        elif style == "dial":
            base.visual(
                Cylinder(radius=0.024, length=0.010),
                origin=Origin(
                    xyz=(front_x + 0.001, y, z_center),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=knob_mat,
                name=f"speed_selector_dial_{i}",
            )
        else:  # lever
            base.visual(
                Box((0.010, 0.016, 0.060)),
                origin=Origin(xyz=(front_x + 0.005, y, z_center)),
                material=knob_mat,
                name=f"speed_selector_lever_{i}",
            )
            base.visual(
                Box((0.008, 0.040, 0.012)),
                origin=Origin(xyz=(front_x + 0.001, y, z_center - 0.018)),
                material=hardware_mat,
                name=f"speed_selector_lever_base_{i}",
            )


def _build_mixer_head(
    head,
    resolved: ResolvedStandMixerConfig,
    *,
    assets: AssetContext,
    head_mat,
    trim_mat,
    hardware_mat,
    knob_mat,
) -> None:
    """Build the mixer_head visuals.

    Local frame: origin is at the head_tilt joint pivot. Head extends along
    local +X from the pivot. Local +Z is up at q=0.
    """
    L = resolved.head_len
    Wh = resolved.head_w
    Hh = resolved.head_h
    shape = resolved.head_shape

    # Main motor housing: a lofted shell with shape-specific cross-sections.
    head_mesh = mesh_from_geometry(
        _build_head_loft_geometry(shape, L, Wh, Hh),
        assets.mesh_path(f"mixer_head_shell_{shape}.obj"),
    )
    head.visual(
        head_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=head_mat,
        name="motor_housing_main",
    )
    if shape == "retro":
        # Retro-style chrome band girdling the head near the shoulder.
        head.visual(
            Cylinder(radius=Hh * 0.55, length=0.012),
            origin=Origin(xyz=(L * 0.30, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_mat,
            name="head_retro_chrome_band",
        )

    # Cooling vent visual ridges (decorations) on the upper side of the head.
    if resolved.detail_level in ("normal", "detailed"):
        for i in range(3):
            head.visual(
                Box((0.005, Wh * 0.55, 0.004)),
                origin=Origin(xyz=(L * 0.38 + i * 0.022, 0.0, Hh * 0.42)),
                material=trim_mat,
                name=f"motor_vent_ridge_{i}",
            )

    # Attachment socket: a downward cylindrical stub at the front of the head.
    socket_x = L * 0.75
    socket_radius = 0.028
    head.visual(
        Cylinder(radius=socket_radius, length=0.030),
        origin=Origin(xyz=(socket_x, 0.0, -Hh * 0.55 + 0.005)),
        material=hardware_mat,
        name="attachment_socket",
    )
    head.visual(
        Cylinder(radius=socket_radius * 1.18, length=0.012),
        origin=Origin(xyz=(socket_x, 0.0, -Hh * 0.55 + 0.022)),
        material=trim_mat,
        name="attachment_socket_collar",
    )

    # Decorative badge on the front of the head.
    if resolved.detail_level in ("normal", "detailed"):
        head.visual(
            Box((0.004, Wh * 0.40, Hh * 0.30)),
            origin=Origin(xyz=(L * 0.96, 0.0, 0.0)),
            material=trim_mat,
            name="head_front_badge",
        )

    # Head-mounted speed selector (when configured).
    if (
        resolved.speed_selector_style != "none"
        and resolved.speed_selector_mount == "head_mounted"
        and resolved.control_count > 0
    ):
        n = resolved.control_count
        style = resolved.speed_selector_style
        side_y = Wh / 2.0 + 0.004
        span = L * 0.40
        if n == 1:
            xs = (L * 0.32,)
        elif n == 2:
            xs = (L * 0.25, L * 0.25 + span * 0.6)
        else:
            xs = (L * 0.22, L * 0.42, L * 0.62)
        for i, hx in enumerate(xs):
            if style == "knob":
                head.visual(
                    Cylinder(radius=0.014, length=0.018),
                    origin=Origin(xyz=(hx, side_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                    material=knob_mat,
                    name=f"speed_selector_knob_{i}",
                )
            elif style == "dial":
                head.visual(
                    Cylinder(radius=0.020, length=0.010),
                    origin=Origin(xyz=(hx, side_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                    material=knob_mat,
                    name=f"speed_selector_dial_{i}",
                )
            else:  # lever
                head.visual(
                    Box((0.020, 0.012, 0.050)),
                    origin=Origin(xyz=(hx, side_y, 0.012)),
                    material=knob_mat,
                    name=f"speed_selector_lever_{i}",
                )


def _xy_section(
    width: float, depth: float, radius: float, z: float, *, x_shift: float = 0.0
) -> list[tuple[float, float, float]]:
    """Rounded-rect cross-section in the XY plane at height z."""
    r = min(radius, 0.499 * min(width, depth))
    return [
        (x + x_shift, y, z) for x, y in rounded_rect_profile(width, depth, r, corner_segments=8)
    ]


def _yz_section(
    width: float, height: float, radius: float, x: float, *, z_shift: float = 0.0
) -> list[tuple[float, float, float]]:
    """Rounded-rect cross-section in the YZ plane at position x."""
    r = min(radius, 0.499 * min(width, height))
    return [
        (x, y, z + z_shift) for y, z in rounded_rect_profile(width, height, r, corner_segments=8)
    ]


def _build_head_loft_geometry(head_shape: HeadShape, L: float, Wh: float, Hh: float) -> object:
    """Lofted motor housing shell. Stations vary by head_shape.

    Local frame: origin at the head_tilt pivot. The head extends along +X
    from x≈0 (rear, pivot zone) to x≈L (front, attachment socket).
    """
    if head_shape == "rounded":
        # Tube-like with rounded oval cross-sections; smooth bulb.
        r = Hh * 0.50
        return section_loft(
            [
                _yz_section(Wh * 0.62, Hh * 0.70, r * 0.70, L * 0.05, z_shift=Hh * 0.05),
                _yz_section(Wh * 0.94, Hh * 0.96, r, L * 0.30, z_shift=Hh * 0.02),
                _yz_section(Wh * 0.96, Hh * 0.92, r, L * 0.65, z_shift=Hh * 0.00),
                _yz_section(Wh * 0.74, Hh * 0.72, r * 0.80, L * 0.92, z_shift=-Hh * 0.04),
            ]
        )
    if head_shape == "boxy":
        # Sharp-edged rectangular housing with small fillet.
        r = min(Wh, Hh) * 0.10
        return section_loft(
            [
                _yz_section(Wh * 0.60, Hh * 0.68, r, L * 0.04),
                _yz_section(Wh * 0.96, Hh * 0.96, r, L * 0.25),
                _yz_section(Wh * 0.96, Hh * 0.96, r, L * 0.75),
                _yz_section(Wh * 0.78, Hh * 0.78, r, L * 0.96, z_shift=-Hh * 0.02),
            ]
        )
    if head_shape == "retro":
        # KitchenAid Artisan: bulbous front, slim rear, sloping topline.
        rb = Hh * 0.46
        return section_loft(
            [
                _yz_section(Wh * 0.46, Hh * 0.56, rb * 0.70, L * 0.03, z_shift=Hh * 0.08),
                _yz_section(Wh * 0.78, Hh * 0.80, rb * 0.85, L * 0.22, z_shift=Hh * 0.04),
                _yz_section(Wh * 0.98, Hh * 1.04, rb, L * 0.55, z_shift=Hh * 0.00),
                _yz_section(Wh * 0.94, Hh * 0.96, rb * 0.95, L * 0.80, z_shift=-Hh * 0.02),
                _yz_section(Wh * 0.70, Hh * 0.66, rb * 0.70, L * 0.96, z_shift=-Hh * 0.06),
            ]
        )
    # industrial — Hobart-style: tall, squarer, gentle taper, vent panel.
    r = min(Wh, Hh) * 0.06
    return section_loft(
        [
            _yz_section(Wh * 0.68, Hh * 0.80, r, L * 0.02),
            _yz_section(Wh * 1.00, Hh * 1.08, r, L * 0.22, z_shift=Hh * 0.02),
            _yz_section(Wh * 1.00, Hh * 1.06, r, L * 0.62, z_shift=Hh * 0.00),
            _yz_section(Wh * 0.92, Hh * 0.94, r, L * 0.88, z_shift=-Hh * 0.04),
            _yz_section(Wh * 0.74, Hh * 0.70, r, L * 0.98, z_shift=-Hh * 0.08),
        ]
    )


def _build_base_loft_geometry(base_shape: BaseShape, D: float, W: float, H: float) -> object:
    """Lofted base shell. Stations stack along Z.

    Local frame: base footprint centered at (0, 0); base extends from z=0
    (floor) to z=H (top). +X is forward, +Y is left.
    """
    if base_shape == "oval":
        # Tall ellipse-like footprint shrinking gently upward.
        rad_low = min(W, D) * 0.48
        rad_mid = min(W, D) * 0.42
        rad_top = min(W, D) * 0.38
        return section_loft(
            [
                _xy_section(D * 1.00, W * 0.96, rad_low, 0.0),
                _xy_section(D * 0.98, W * 0.94, rad_low, H * 0.30),
                _xy_section(D * 0.94, W * 0.90, rad_mid, H * 0.70),
                _xy_section(D * 0.86, W * 0.82, rad_top, H * 1.00),
            ]
        )
    if base_shape == "rectangle":
        # Slab with sharp corners.
        r = min(W, D) * 0.06
        return section_loft(
            [
                _xy_section(D, W, r, 0.0),
                _xy_section(D * 0.99, W * 0.99, r, H * 0.50),
                _xy_section(D * 0.96, W * 0.96, r, H * 1.00),
            ]
        )
    # rounded — medium fillet with gentle taper.
    r = min(W, D) * 0.22
    return section_loft(
        [
            _xy_section(D, W, r, 0.0),
            _xy_section(D * 0.97, W * 0.97, r, H * 0.40),
            _xy_section(D * 0.90, W * 0.92, r * 0.95, H * 0.80),
            _xy_section(D * 0.82, W * 0.86, r * 0.85, H * 1.00),
        ]
    )


def _build_bowl_lathe_geometry(
    outer_r: float,
    depth: float,
    bowl_shape: BowlShape,
) -> LatheGeometry:
    """Construct a real hollow bowl shell via lathing outer + inner profiles.

    Profile proportions follow `rec_stand_mixer_3452cb*` (5-star reference):
    inner top z is only ~5% of depth below outer top z (a thin lip), so the
    cavity opening is clearly visible from above.
    """
    if bowl_shape == "deep_bowl":
        # Reference proportions (gently curved, deep bowl).
        outer_profile = [
            (outer_r * 0.19, 0.000 * depth),
            (outer_r * 0.36, 0.044 * depth),
            (outer_r * 0.67, 0.221 * depth),
            (outer_r * 0.89, 0.537 * depth),
            (outer_r * 1.00, 0.853 * depth),
            (outer_r * 0.975, 1.000 * depth),
        ]
        inner_profile = [
            (outer_r * 0.000, 0.029 * depth),
            (outer_r * 0.288, 0.088 * depth),
            (outer_r * 0.610, 0.250 * depth),
            (outer_r * 0.839, 0.559 * depth),
            (outer_r * 0.941, 0.950 * depth),
        ]
    elif bowl_shape == "wide_bowl":
        # Broader belly, larger max radius, slightly shorter overall.
        outer_profile = [
            (outer_r * 0.32, 0.000 * depth),
            (outer_r * 0.58, 0.080 * depth),
            (outer_r * 0.84, 0.300 * depth),
            (outer_r * 1.00, 0.620 * depth),
            (outer_r * 1.04, 0.880 * depth),
            (outer_r * 0.985, 1.000 * depth),
        ]
        inner_profile = [
            (outer_r * 0.000, 0.040 * depth),
            (outer_r * 0.460, 0.135 * depth),
            (outer_r * 0.770, 0.330 * depth),
            (outer_r * 0.950, 0.660 * depth),
            (outer_r * 0.955, 0.950 * depth),
        ]
    else:  # tapered (V-shape: narrow flat bottom, broad rim)
        outer_profile = [
            (outer_r * 0.22, 0.000 * depth),
            (outer_r * 0.34, 0.110 * depth),
            (outer_r * 0.55, 0.400 * depth),
            (outer_r * 0.78, 0.700 * depth),
            (outer_r * 0.94, 0.910 * depth),
            (outer_r * 0.975, 1.000 * depth),
        ]
        inner_profile = [
            (outer_r * 0.000, 0.045 * depth),
            (outer_r * 0.220, 0.150 * depth),
            (outer_r * 0.460, 0.430 * depth),
            (outer_r * 0.730, 0.720 * depth),
            (outer_r * 0.910, 0.950 * depth),
        ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        end_cap="round",
        lip_samples=10,
    )


def _build_whisk_geometry(body_height: float) -> CylinderGeometry:
    """Wire whisk: spline-swept wire loops around a central spine."""
    s = body_height / 0.116  # reference body length ≈ 0.116 m
    geom = CylinderGeometry(radius=0.0080, height=0.024 * s)
    geom.translate(0.0, 0.0, -0.014 * s)
    geom.merge(CylinderGeometry(radius=0.0120, height=0.018 * s).translate(0.0, 0.0, -0.032 * s))
    geom.merge(
        CylinderGeometry(radius=0.0100, height=0.016 * s).translate(
            0.0, 0.0, -(body_height - 0.008)
        )
    )
    loop_count = 8
    r_lobe = body_height * 0.30
    z_top = -0.042 * s
    z_bot = -(body_height - 0.005)
    for index in range(loop_count):
        angle = (math.pi * index) / loop_count
        c = math.cos(angle)
        ss = math.sin(angle)
        wire = tube_from_spline_points(
            [
                (0.010 * c, 0.010 * ss, z_top),
                (r_lobe * c * 0.55, r_lobe * ss * 0.55, z_top - body_height * 0.18),
                (r_lobe * c, r_lobe * ss, (z_top + z_bot) * 0.5),
                (r_lobe * c * 0.55, r_lobe * ss * 0.55, z_bot + body_height * 0.18),
                (0.0, 0.0, z_bot),
                (-r_lobe * c * 0.55, -r_lobe * ss * 0.55, z_bot + body_height * 0.18),
                (-r_lobe * c, -r_lobe * ss, (z_top + z_bot) * 0.5),
                (-r_lobe * c * 0.55, -r_lobe * ss * 0.55, z_top - body_height * 0.18),
                (-0.010 * c, -0.010 * ss, z_top),
            ],
            radius=0.0015,
            samples_per_segment=12,
            radial_segments=10,
            cap_ends=True,
        )
        geom.merge(wire)
    return geom


def _build_dough_hook_geometry(body_height: float) -> CylinderGeometry:
    """C-shaped curved dough hook: spline tube along a hooked path."""
    s = body_height / 0.132
    geom = CylinderGeometry(radius=0.008, height=0.030 * s).translate(0.0, 0.0, -0.015 * s)
    geom.merge(CylinderGeometry(radius=0.012, height=0.014 * s).translate(0.0, 0.0, -0.037 * s))
    curve = tube_from_spline_points(
        [
            (0.000, 0.000, -0.044 * s),
            (0.010 * s, 0.000, -0.060 * s),
            (0.025 * s, 0.000, -0.080 * s),
            (0.032 * s, 0.000, -0.102 * s),
            (0.027 * s, 0.000, -0.118 * s),
            (0.014 * s, 0.000, -0.128 * s),
            (-0.002 * s, 0.000, -body_height),
        ],
        radius=0.0068,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    geom.merge(curve)
    return geom


def _build_flat_beater_geometry(body_height: float) -> CylinderGeometry:
    """Leaf-shaped paddle with center cutout (real flat beater)."""
    s = body_height / 0.142
    outer = sample_catmull_rom_spline_2d(
        [
            (0.016 * s, 0.024 * s),
            (0.040 * s, -0.002 * s),
            (0.050 * s, -0.035 * s),
            (0.044 * s, -0.078 * s),
            (0.026 * s, -0.108 * s),
            (0.000, -0.118 * s),
            (-0.026 * s, -0.108 * s),
            (-0.044 * s, -0.078 * s),
            (-0.050 * s, -0.035 * s),
            (-0.040 * s, -0.002 * s),
            (-0.016 * s, 0.024 * s),
        ],
        samples_per_segment=10,
        closed=True,
    )
    inner = sample_catmull_rom_spline_2d(
        [
            (0.010 * s, 0.002 * s),
            (0.022 * s, -0.020 * s),
            (0.025 * s, -0.062 * s),
            (0.014 * s, -0.093 * s),
            (0.000, -0.102 * s),
            (-0.014 * s, -0.093 * s),
            (-0.025 * s, -0.062 * s),
            (-0.022 * s, -0.020 * s),
            (-0.010 * s, 0.002 * s),
        ],
        samples_per_segment=10,
        closed=True,
    )
    paddle = ExtrudeWithHolesGeometry(outer, [inner], 0.006, center=True)
    paddle.rotate_x(math.pi / 2.0).translate(0.0, 0.0, -body_height * 0.62)
    shaft = CylinderGeometry(radius=0.0065, height=body_height * 0.36).translate(
        0.0, 0.0, -body_height * 0.20
    )
    shaft.merge(
        CylinderGeometry(radius=0.010, height=0.026 * s).translate(0.0, 0.0, -body_height * 0.34)
    )
    shaft.merge(paddle)
    return shaft


def _build_tool_attachment(
    tool,
    resolved: ResolvedStandMixerConfig,
    *,
    assets: AssetContext,
    tool_mat,
    hardware_mat,
) -> None:
    """Build the tool_attachment shape under the head.

    Local frame: origin is at the attachment_socket exit on the head. Tool
    extends along local -Z (down). At q=0 the tool spins about local Z.
    """
    drop = resolved.tool_drop_z
    kind = resolved.tool_type

    # Connector shaft from socket to tool body (kept as a discrete visual so the
    # validator's allow_overlap rules continue to reference it by name).
    shaft_len = drop * 0.22
    tool.visual(
        Cylinder(radius=0.010, length=shaft_len),
        origin=Origin(xyz=(0.0, 0.0, -shaft_len / 2.0)),
        material=hardware_mat,
        name="tool_shaft",
    )

    body_height = drop - shaft_len

    if kind == "flat_beater":
        mesh = mesh_from_geometry(
            _build_flat_beater_geometry(body_height),
            assets.mesh_path("flat_beater_blade.obj"),
        )
        tool.visual(
            mesh,
            origin=Origin(xyz=(0.0, 0.0, -shaft_len)),
            material=tool_mat,
            name="flat_beater_blade_main",
        )
    elif kind == "whisk":
        mesh = mesh_from_geometry(
            _build_whisk_geometry(body_height),
            assets.mesh_path("wire_whisk.obj"),
        )
        tool.visual(
            mesh,
            origin=Origin(xyz=(0.0, 0.0, -shaft_len)),
            material=tool_mat,
            name="whisk_assembly",
        )
    else:  # dough_hook
        mesh = mesh_from_geometry(
            _build_dough_hook_geometry(body_height),
            assets.mesh_path("dough_hook_curve.obj"),
        )
        tool.visual(
            mesh,
            origin=Origin(xyz=(0.0, 0.0, -shaft_len)),
            material=tool_mat,
            name="dough_hook_upper_shaft",
        )


def _build_bowl(
    bowl_part,
    resolved: ResolvedStandMixerConfig,
    *,
    assets: AssetContext,
    bowl_mat,
    trim_mat,
) -> None:
    """Build the mixing_bowl visuals.

    Local frame: origin at the BOTTOM of the bowl (outside surface). Bowl
    extends upward along +Z.
    """
    ro = resolved.bowl_outer_r
    h = resolved.bowl_depth

    # Real hollow shell: outer + inner profiles lathed into a true cavity.
    # The end_cap="round" parameter inside the lathe geometry already builds
    # the rounded rim/lip connecting outer top to inner top — no extra rim
    # disc, no decorative side handle (would float free of the shell).
    bowl_mesh = mesh_from_geometry(
        _build_bowl_lathe_geometry(ro, h, resolved.bowl_shape),
        assets.mesh_path(f"mixing_bowl_shell_{resolved.bowl_shape}.obj"),
    )
    bowl_part.visual(
        bowl_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bowl_mat,
        name="bowl_outer",
    )


def _build_carriage_visuals(
    carriage,
    resolved: ResolvedStandMixerConfig,
    *,
    body_mat,
    trim_mat,
) -> None:
    """Build a thin platter/dish-shape carriage that the bowl sits on.

    Local frame: origin at the center of the carriage top surface, on the
    bottom plane of the bowl (z=0 in carriage frame).
    """
    r = resolved.bowl_outer_r * 1.12
    h = CARRIAGE_HEIGHT_Z
    carriage.visual(
        Cylinder(radius=r, length=h),
        origin=Origin(xyz=(0.0, 0.0, -h / 2.0)),
        material=body_mat,
        name="bowl_carriage_plate",
    )
    carriage.visual(
        Cylinder(radius=r * 0.92, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=trim_mat,
        name="bowl_carriage_lip",
    )


def _build_head_lock_visuals(lock_part, *, hardware_mat) -> None:
    """A small lever lock attached to the head."""
    lock_part.visual(
        Box((0.030, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hardware_mat,
        name="head_lock_body",
    )
    lock_part.visual(
        Box((0.040, 0.010, 0.008)),
        origin=Origin(xyz=(0.015, 0.0, 0.012)),
        material=hardware_mat,
        name="head_lock_lever",
    )


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------


def build_stand_mixer(
    config: StandMixerConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    config = config or StandMixerConfig()
    resolved = resolve_config(config)
    if assets is None:
        assets = AssetContext(Path(tempfile.mkdtemp(prefix="articraft-stand-mixer-assets-")))
    model = ArticulatedObject(name=resolved.name, assets=assets)

    palette = MATERIAL_PALETTES[resolved.material_style]
    body_mat = model.material("mixer_body", rgba=palette["body"])
    trim_mat = model.material("mixer_trim", rgba=palette["trim"])
    head_mat = model.material("mixer_head", rgba=palette["head"])
    bowl_mat = model.material("mixer_bowl", rgba=palette["bowl"])
    tool_mat = model.material("mixer_tool", rgba=palette["tool"])
    hardware_mat = model.material("mixer_hardware", rgba=palette["hardware"])
    knob_mat = model.material("mixer_knob", rgba=palette["knob"])

    # --- Base + column ------------------------------------------------------
    base = model.part("mixer_base")
    _build_base_body(base, resolved, assets=assets, body_mat=body_mat, trim_mat=trim_mat)
    _build_rear_column(base, resolved, body_mat=body_mat, trim_mat=trim_mat)
    if resolved.speed_selector_mount == "base_fixed":
        _add_speed_selector_on_base(
            base,
            resolved,
            hardware_mat=hardware_mat,
            knob_mat=knob_mat,
        )

    # --- Head --------------------------------------------------------------
    head = model.part("mixer_head")
    _build_mixer_head(
        head,
        resolved,
        assets=assets,
        head_mat=head_mat,
        trim_mat=trim_mat,
        hardware_mat=hardware_mat,
        knob_mat=knob_mat,
    )

    # head_tilt_joint: revolute about (0, -1, 0). Positive q lifts the front
    # of the head upward away from the bowl. Range 0..head_tilt_range_rad.
    model.articulation(
        "head_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(resolved.head_pivot_x, 0.0, resolved.head_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=resolved.head_tilt_range_rad,
        ),
    )

    # --- Tool attachment (continuous rotor under the head) -----------------
    tool = model.part("tool_attachment")
    _build_tool_attachment(
        tool, resolved, assets=assets, tool_mat=tool_mat, hardware_mat=hardware_mat
    )

    # Tool joint origin: at the attachment_socket exit on the head, expressed
    # in the head's local frame.
    socket_x_local = resolved.head_len * 0.75
    socket_z_local = -resolved.head_h * 0.55 - 0.010
    model.articulation(
        "tool_spin_joint",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=tool,
        origin=Origin(xyz=(socket_x_local, 0.0, socket_z_local)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )

    # --- Bowl + optional carriage ------------------------------------------
    # The bowl center in WORLD x sits under the tool spin axis. The tool is
    # attached to the head, and at q=0 the head tilt is zero so the world
    # x-position of the tool axis equals head_pivot_x + socket_x_local.
    bowl_world_x = resolved.head_pivot_x + socket_x_local
    bowl_world_z = resolved.base_h  # bowl sits on top of base surface

    if resolved.bowl_lift_type == "none":
        # bowl is non-articulating; mount it via FIXED articulation so the
        # tree connectivity is well-defined.
        bowl = model.part("mixing_bowl")
        _build_bowl(bowl, resolved, assets=assets, bowl_mat=bowl_mat, trim_mat=trim_mat)
        # Visible saucer/platter on the base under the bowl so the bowl does
        # not visually float on its narrow foot (especially for tapered shape).
        base.visual(
            Cylinder(radius=resolved.bowl_outer_r * 1.06, length=0.005),
            origin=Origin(xyz=(bowl_world_x, 0.0, bowl_world_z - 0.0025)),
            material=trim_mat,
            name="bowl_seat_platter",
        )
        model.articulation(
            "bowl_to_base_fixed",
            ArticulationType.FIXED,
            parent=base,
            child=bowl,
            origin=Origin(xyz=(bowl_world_x, 0.0, bowl_world_z)),
        )
    elif resolved.bowl_lift_type == "slide_horizontal":
        # Carriage slides; bowl is fixed to the carriage.
        carriage = model.part("bowl_carriage")
        _build_carriage_visuals(carriage, resolved, body_mat=body_mat, trim_mat=trim_mat)

        # Choose slide axis. The joint frame is the world frame (no rpy).
        if resolved.bowl_slide_axis == "x":
            axis = (1.0, 0.0, 0.0)
        else:
            axis = (0.0, 1.0, 0.0)
        slide_lower = -resolved.bowl_slide_range / 2.0
        slide_upper = +resolved.bowl_slide_range / 2.0

        model.articulation(
            "bowl_lift_or_slide_joint",
            ArticulationType.PRISMATIC,
            parent=base,
            child=carriage,
            origin=Origin(xyz=(bowl_world_x, 0.0, bowl_world_z)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=0.20,
                lower=slide_lower,
                upper=slide_upper,
            ),
        )

        bowl = model.part("mixing_bowl")
        _build_bowl(bowl, resolved, assets=assets, bowl_mat=bowl_mat, trim_mat=trim_mat)
        # Bowl fixed on top of carriage at carriage local origin.
        model.articulation(
            "bowl_to_carriage_fixed",
            ArticulationType.FIXED,
            parent=carriage,
            child=bowl,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
    else:  # lever_lift
        # Carriage rides on a lever that lifts/lowers the bowl along world Z.
        # Positive q raises the bowl toward the tool. At q=0 the bowl sits
        # lowered by lever_lift_travel below the upper position.
        carriage = model.part("bowl_carriage")
        _build_carriage_visuals(carriage, resolved, body_mat=body_mat, trim_mat=trim_mat)

        # The carriage at q=0 is in the LOWERED position. The lower position
        # sits lever_lift_travel below the docked top. Place the carriage so
        # that at full upper, the bowl rests on top of the base surface.
        lowered_z = bowl_world_z - resolved.lever_lift_travel
        model.articulation(
            "bowl_lift_or_slide_joint",
            ArticulationType.PRISMATIC,
            parent=base,
            child=carriage,
            origin=Origin(xyz=(bowl_world_x, 0.0, lowered_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=0.20,
                lower=0.0,
                upper=resolved.lever_lift_travel,
            ),
        )

        # Lever arm decoration on the base side (visual on base) so it reads
        # like a lever_lift mechanism.
        base.visual(
            Box((0.060, 0.014, 0.010)),
            origin=Origin(
                xyz=(bowl_world_x - resolved.bowl_outer_r * 0.6, 0.0, resolved.base_h - 0.002),
            ),
            material=hardware_mat,
            name="bowl_lift_lever",
        )

        bowl = model.part("mixing_bowl")
        _build_bowl(bowl, resolved, assets=assets, bowl_mat=bowl_mat, trim_mat=trim_mat)
        model.articulation(
            "bowl_to_carriage_fixed",
            ArticulationType.FIXED,
            parent=carriage,
            child=bowl,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

    # --- Optional head_lock_control ----------------------------------------
    if resolved.has_head_lock:
        lock_part = model.part("head_lock_control")
        _build_head_lock_visuals(lock_part, hardware_mat=hardware_mat)
        # Mount on the side of the head with a revolute lock joint.
        model.articulation(
            "head_lock_joint",
            ArticulationType.REVOLUTE,
            parent=head,
            child=lock_part,
            origin=Origin(
                xyz=(resolved.head_len * 0.18, resolved.head_w / 2.0 + 0.004, 0.0),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.0),
        )

    # --- Optional speed_selector_joint (rotary control) --------------------
    # A speed selector knob/dial that rotates is a separate revolute child.
    # We attach it on the surface where the visual decoration lives (front
    # face of base or side of head). When style is "none", skip entirely.
    if resolved.speed_selector_style in ("knob", "dial") and resolved.control_count > 0:
        # Single articulating control (the first one).
        selector_part = model.part("speed_selector")
        if resolved.speed_selector_mount == "base_fixed":
            # Mount on +X face of the base.
            front_x = resolved.base_d / 2.0 + 0.012
            mount_origin = Origin(
                xyz=(front_x, 0.0, resolved.base_h * 0.62),
                rpy=(0.0, math.pi / 2.0, 0.0),
            )
            parent_part = base
            axis = (0.0, 0.0, 1.0)
        else:
            # Mount on +Y face of the head.
            mount_origin = Origin(
                xyz=(resolved.head_len * 0.32, resolved.head_w / 2.0 + 0.012, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            )
            parent_part = head
            axis = (0.0, 0.0, 1.0)

        # Knob visual: a small disc lying flat in its own local frame
        # (cylinder axis along child-local Z, which after the mount rpy
        # protrudes outward from the host face).
        knob_r = 0.020
        knob_h = 0.014
        selector_part.visual(
            Cylinder(radius=knob_r, length=knob_h),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=knob_mat,
            name="speed_selector_knob_geom",
        )
        # Indicator: a raised radial tick on the FRONT face of the knob,
        # spanning from near the center to the rim and protruding outward in
        # +Z. This way slider rotations are immediately visible — earlier
        # version had the box buried inside the cylinder.
        indicator_len = knob_r * 0.85
        selector_part.visual(
            Box((indicator_len, 0.004, 0.005)),
            origin=Origin(xyz=(indicator_len / 2.0 + 0.001, 0.0, knob_h / 2.0 + 0.003)),
            material=trim_mat,
            name="speed_selector_indicator",
        )

        model.articulation(
            "speed_selector_joint",
            ArticulationType.CONTINUOUS,
            parent=parent_part,
            child=selector_part,
            origin=mount_origin,
            axis=axis,
            motion_limits=MotionLimits(effort=1.0, velocity=4.0),
        )

    return model


def build_seeded_stand_mixer(seed: int, *, assets: AssetContext | None = None) -> ArticulatedObject:
    return build_stand_mixer(config_from_seed(seed), assets=assets)


def with_overrides(config: StandMixerConfig, **kwargs: object) -> StandMixerConfig:
    return replace(config, **kwargs)


# ---------------------------------------------------------------------------
# Validator: prompt-specific checks. Compiler-owned baseline checks run
# separately.
# ---------------------------------------------------------------------------


def run_stand_mixer_tests(object_model: ArticulatedObject, config: StandMixerConfig) -> TestReport:
    resolved = resolve_config(config)
    ctx = TestContext(object_model)
    base = object_model.get_part("mixer_base")
    head = object_model.get_part("mixer_head")
    tool = object_model.get_part("tool_attachment")
    bowl = object_model.get_part("mixing_bowl")

    # --- head_tilt_joint is revolute ----------------------------------------
    head_joint = object_model.get_articulation("head_tilt_joint")
    ctx.check(
        "head_tilt_joint_is_revolute",
        head_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={head_joint.articulation_type}",
    )
    # Pivot must sit at the rear of the base envelope (negative X side).
    ctx.check(
        "head_pivot_at_rear",
        head_joint.origin.xyz[0] < 0.0,
        details=f"head_pivot_x={head_joint.origin.xyz[0]}",
    )
    # Range within [25, 60] degrees per spec.
    upper = head_joint.motion_limits.upper if head_joint.motion_limits else None
    ctx.check(
        "head_tilt_range_in_spec",
        upper is not None and math.radians(25.0) - 1e-3 <= upper <= math.radians(60.0) + 1e-3,
        details=f"upper={upper}",
    )

    # --- tool_spin_joint is continuous --------------------------------------
    tool_joint = object_model.get_articulation("tool_spin_joint")
    ctx.check(
        "tool_spin_joint_is_continuous",
        tool_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={tool_joint.articulation_type}",
    )
    # Tool spin axis should be approximately vertical (parallel to world Z at
    # q=0 of head_tilt). The joint is defined in the head's local frame; the
    # head's local +Z aligns with world +Z at head_tilt=0, so axis=(0,0,1) is
    # vertical. We assert the configured axis numerically.
    axis = tool_joint.axis
    ctx.check(
        "tool_spin_axis_vertical_local",
        abs(axis[2]) > 0.95 and abs(axis[0]) < 0.2 and abs(axis[1]) < 0.2,
        details=f"axis={axis}",
    )

    # --- tool above bowl center, head above bowl ----------------------------
    tool_aabb = ctx.part_world_aabb(tool)
    bowl_aabb = ctx.part_world_aabb(bowl)
    head_aabb = ctx.part_world_aabb(head)
    base_aabb = ctx.part_world_aabb(base)

    # Tool x and y centered roughly above bowl center.
    if tool_aabb is not None and bowl_aabb is not None:
        tool_cx = (tool_aabb[0][0] + tool_aabb[1][0]) / 2.0
        tool_cy = (tool_aabb[0][1] + tool_aabb[1][1]) / 2.0
        bowl_cx = (bowl_aabb[0][0] + bowl_aabb[1][0]) / 2.0
        bowl_cy = (bowl_aabb[0][1] + bowl_aabb[1][1]) / 2.0
        ctx.check(
            "tool_above_bowl_center_xy",
            abs(tool_cx - bowl_cx) < resolved.bowl_outer_r * 0.5
            and abs(tool_cy - bowl_cy) < resolved.bowl_outer_r * 0.5,
            details=(
                f"tool_xy=({tool_cx:.3f},{tool_cy:.3f}) bowl_xy=({bowl_cx:.3f},{bowl_cy:.3f})"
            ),
        )

    # Bowl below head (head bottom z > bowl bottom z, and bowl top within head
    # zone).
    if head_aabb is not None and bowl_aabb is not None:
        ctx.check(
            "bowl_under_head",
            head_aabb[0][2] > bowl_aabb[0][2] - 0.005,
            details=f"head_min_z={head_aabb[0][2]}, bowl_min_z={bowl_aabb[0][2]}",
        )

    # Bowl above base top (its bottom near the top of the base envelope).
    if bowl_aabb is not None and base_aabb is not None:
        ctx.check(
            "bowl_above_base_top",
            bowl_aabb[0][2] >= resolved.base_h - 0.060,
            details=f"bowl_min_z={bowl_aabb[0][2]} base_h={resolved.base_h}",
        )

    # --- bowl motion check ---------------------------------------------------
    if resolved.bowl_lift_type != "none":
        lift_joint = object_model.get_articulation("bowl_lift_or_slide_joint")
        ctx.check(
            "bowl_lift_joint_is_prismatic",
            lift_joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={lift_joint.articulation_type}",
        )
        if resolved.bowl_lift_type == "lever_lift":
            # Axis should be vertical world Z.
            la = lift_joint.axis
            ctx.check(
                "lever_lift_axis_vertical",
                abs(la[2]) > 0.95,
                details=f"axis={la}",
            )
        else:
            la = lift_joint.axis
            ctx.check(
                "slide_horizontal_axis_horizontal",
                abs(la[2]) < 0.05,
                details=f"axis={la}",
            )

    # --- head sweep does not pierce column or bowl --------------------------
    # At full tilt, the head should rise away from the bowl. Sample head AABB
    # at upper limit and check its bottom Z exceeds rest pose.
    if head_joint.motion_limits is not None and head_joint.motion_limits.upper is not None:
        rest_head_aabb = ctx.part_world_aabb(head)
        with ctx.pose({head_joint: head_joint.motion_limits.upper}):
            tilted_head_aabb = ctx.part_world_aabb(head)
        ctx.check(
            "head_tilts_upward",
            rest_head_aabb is not None
            and tilted_head_aabb is not None
            and tilted_head_aabb[1][2] > rest_head_aabb[1][2] - 0.005,
            details=f"rest={rest_head_aabb}, tilted={tilted_head_aabb}",
        )

    # --- control attached ----------------------------------------------------
    if resolved.speed_selector_style != "none" and resolved.control_count > 0:
        # If the selector is a knob/dial it should be an articulated part.
        if resolved.speed_selector_style in ("knob", "dial"):
            selector_joint = object_model.get_articulation("speed_selector_joint")
            ctx.check(
                "speed_selector_joint_is_continuous",
                selector_joint.articulation_type == ArticulationType.CONTINUOUS,
                details=f"type={selector_joint.articulation_type}",
            )
        # else: lever or no-articulation control attached as visual on parent.

    # --- explicit allowances for legitimate nested fits --------------------
    # Tool shaft nests into the attachment_socket on the head.
    ctx.allow_overlap(
        head,
        tool,
        elem_a="attachment_socket",
        elem_b="tool_shaft",
        reason="The tool shaft is captured inside the attachment socket on the head.",
    )
    ctx.allow_overlap(
        head,
        tool,
        elem_a="attachment_socket_collar",
        elem_b="tool_shaft",
        reason="The collar wraps the tool shaft entry.",
    )

    # Carriage and bowl: the bowl sits on the carriage; carriage lip overlaps
    # the bowl outer at the seating ring.
    if resolved.bowl_lift_type != "none":
        carriage = object_model.get_part("bowl_carriage")
        ctx.allow_overlap(
            carriage,
            bowl,
            reason="The bowl is seated on the carriage; small intentional seat overlap.",
        )

    # Rear column pivot cap straddles the head pivot zone; allow gentle nest
    # with the motor housing at q=0.
    ctx.allow_overlap(
        base,
        head,
        elem_a="rear_column_pivot_cap",
        elem_b="motor_housing_main",
        reason="The head pivot cap is shared by the column and the head at the pivot.",
    )

    return ctx.report()
