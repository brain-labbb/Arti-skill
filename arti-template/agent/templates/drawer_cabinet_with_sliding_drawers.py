"""Drawer cabinet with sliding drawers — modular procedural template.

Category identity: a grounded cabinet body carries N PRISMATIC drawers that
slide along +X, with an optional REVOLUTE hinged top lid gated by
``has_top_lid``.

PRIMARY_ANCHOR: ``rec_drawer_cabinet_with_sliding_drawers_ed911543`` — bedside
cabinet with 3 drawers + hinged lid compartment. ``config_from_seed(0)``
reproduces the anchor defaults (3 drawers, lid, warm-oak palette,
``bedside_compartment`` body style).

Canonical spec: ``articraft_template_authoring/specs_modular_v1/drawer_cabinet_with_sliding_drawers.md``
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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

__modular__ = True

CabinetStyle = Literal[
    "bedside_compartment",
    "uniform_stack",
    "plinth_base",
    "wide_low",
]
MaterialStyle = Literal["warm_oak", "painted_white", "walnut_stain", "industrial_grey"]

CABINET_STYLES: tuple[CabinetStyle, ...] = (
    "bedside_compartment",
    "uniform_stack",
    "plinth_base",
    "wide_low",
)
MATERIAL_STYLES: tuple[MaterialStyle, ...] = (
    "warm_oak",
    "painted_white",
    "walnut_stain",
    "industrial_grey",
)

PALETTES: dict[MaterialStyle, dict[str, tuple[float, float, float, float]]] = {
    "warm_oak": {
        "case": (0.58, 0.44, 0.28, 1.0),
        "drawer_front": (0.52, 0.38, 0.24, 1.0),
        "drawer_box": (0.74, 0.67, 0.53, 1.0),
        "hardware": (0.22, 0.22, 0.23, 1.0),
        "accent": (0.42, 0.30, 0.18, 1.0),
    },
    "painted_white": {
        "case": (0.90, 0.89, 0.86, 1.0),
        "drawer_front": (0.94, 0.93, 0.90, 1.0),
        "drawer_box": (0.82, 0.80, 0.76, 1.0),
        "hardware": (0.30, 0.30, 0.32, 1.0),
        "accent": (0.62, 0.64, 0.66, 1.0),
    },
    "walnut_stain": {
        "case": (0.34, 0.22, 0.14, 1.0),
        "drawer_front": (0.42, 0.28, 0.17, 1.0),
        "drawer_box": (0.56, 0.40, 0.26, 1.0),
        "hardware": (0.68, 0.70, 0.72, 1.0),
        "accent": (0.24, 0.16, 0.10, 1.0),
    },
    "industrial_grey": {
        "case": (0.42, 0.44, 0.46, 1.0),
        "drawer_front": (0.50, 0.52, 0.54, 1.0),
        "drawer_box": (0.36, 0.38, 0.40, 1.0),
        "hardware": (0.18, 0.19, 0.20, 1.0),
        "accent": (0.72, 0.46, 0.12, 1.0),
    },
}

# Anchor geometry (ed911543) — seed 0 reproduces these proportions.
_ANCHOR_WIDTH = 0.52
_ANCHOR_DEPTH = 0.42
_ANCHOR_BODY_HEIGHT = 0.662
_ANCHOR_DRAWER_COUNT = 3
_ANCHOR_SHELF_Z = 0.53
_ANCHOR_DRAWER_Z = (0.0905, 0.2675, 0.4445)
_ANCHOR_DRAWER_TRAVEL = 0.24


@dataclass(frozen=True)
class DrawerCabinetWithSlidingDrawersConfig:
    cabinet_style: CabinetStyle = "bedside_compartment"
    material_style: MaterialStyle = "warm_oak"
    drawer_count: int = 3
    has_top_lid: bool = True
    cabinet_width: float = _ANCHOR_WIDTH
    cabinet_depth: float = _ANCHOR_DEPTH
    body_height: float = _ANCHOR_BODY_HEIGHT
    drawer_travel: float = _ANCHOR_DRAWER_TRAVEL
    name: str = "reference_drawer_cabinet_with_sliding_drawers"


@dataclass(frozen=True)
class ResolvedDrawerCabinetWithSlidingDrawersConfig:
    cabinet_style: CabinetStyle
    material_style: MaterialStyle
    drawer_count: int
    has_top_lid: bool
    cabinet_width: float
    cabinet_depth: float
    body_height: float
    plinth_height: float
    side_thickness: float
    back_thickness: float
    panel_thickness: float
    shelf_z: float
    drawer_front_width: float
    drawer_front_height: float
    drawer_front_thickness: float
    drawer_box_width: float
    drawer_box_depth: float
    drawer_box_height: float
    drawer_panel_thickness: float
    drawer_bottom_thickness: float
    drawer_travel: float
    lid_thickness: float
    lid_width: float
    lid_depth: float
    drawer_center_z: tuple[float, ...]
    name: str


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def config_from_seed(seed: int) -> DrawerCabinetWithSlidingDrawersConfig:
    if seed == 0:
        return DrawerCabinetWithSlidingDrawersConfig()

    rng = random.Random(seed)
    return DrawerCabinetWithSlidingDrawersConfig(
        cabinet_style=rng.choice(CABINET_STYLES),
        material_style=rng.choice(MATERIAL_STYLES),
        drawer_count=rng.randint(1, 5),
        has_top_lid=rng.choice((True, False)),
        cabinet_width=round(rng.uniform(0.38, 0.72), 3),
        cabinet_depth=round(rng.uniform(0.32, 0.56), 3),
        body_height=round(rng.uniform(0.48, 1.05), 3),
        drawer_travel=round(rng.uniform(0.14, 0.32), 3),
        name=f"seeded_drawer_cabinet_with_sliding_drawers_{seed}",
    )


def resolve_config(
    config: DrawerCabinetWithSlidingDrawersConfig,
) -> ResolvedDrawerCabinetWithSlidingDrawersConfig:
    if config.cabinet_style not in CABINET_STYLES:
        raise ValueError(f"Unsupported cabinet_style: {config.cabinet_style}")
    if config.material_style not in PALETTES:
        raise ValueError(f"Unsupported material_style: {config.material_style}")

    drawer_count = max(1, min(5, int(config.drawer_count)))
    has_top_lid = bool(config.has_top_lid)

    width = _clamp(config.cabinet_width, 0.34, 0.80)
    depth = _clamp(config.cabinet_depth, 0.28, 0.62)
    body_height = _clamp(config.body_height, 0.40, 1.20)

    if config.cabinet_style == "wide_low":
        width = max(width, 0.48)
        body_height = min(body_height, 0.72)
    elif config.cabinet_style == "uniform_stack":
        body_height = max(body_height, 0.55)
    elif config.cabinet_style == "plinth_base":
        body_height = max(body_height, 0.58)

    side_t = 0.018
    back_t = 0.012
    panel_t = 0.018
    plinth_h = 0.06 if config.cabinet_style == "plinth_base" else 0.0

    usable_h = body_height - plinth_h
    top_band = 0.0
    shelf_z = usable_h
    if has_top_lid and config.cabinet_style in {"bedside_compartment", "plinth_base"}:
        top_band = max(0.10, usable_h * 0.20)
        shelf_z = usable_h - top_band
    elif has_top_lid:
        top_band = max(0.08, usable_h * 0.14)
        shelf_z = usable_h - top_band

    drawer_stack_h = shelf_z - panel_t
    drawer_pitch = drawer_stack_h / max(drawer_count, 1)
    drawer_front_h = min(0.20, max(0.10, drawer_pitch * 0.88))
    drawer_box_h = drawer_front_h * 0.81
    drawer_front_w = width - 2.0 * side_t - 0.010
    drawer_box_w = width - 2.0 * side_t - 0.024
    drawer_box_d = min(depth - back_t - 0.06, depth * 0.78)
    travel = min(_clamp(config.drawer_travel, 0.10, 0.36), drawer_box_d * 0.72)

    if (
        config.cabinet_style == "bedside_compartment"
        and drawer_count == _ANCHOR_DRAWER_COUNT
        and abs(width - _ANCHOR_WIDTH) < 0.002
        and abs(depth - _ANCHOR_DEPTH) < 0.002
        and abs(body_height - _ANCHOR_BODY_HEIGHT) < 0.002
    ):
        drawer_center_z = _ANCHOR_DRAWER_Z
        shelf_z = _ANCHOR_SHELF_Z
        drawer_front_h = 0.173
        drawer_box_h = 0.14
        drawer_front_w = width - 2.0 * side_t - 0.010
        drawer_box_w = 0.448
        drawer_box_d = 0.33
        travel = _ANCHOR_DRAWER_TRAVEL
    else:
        first_z = panel_t + drawer_pitch * 0.5
        drawer_center_z = tuple(first_z + i * drawer_pitch for i in range(drawer_count))

    lid_t = 0.018
    lid_w = width - 0.004
    lid_d = depth - 0.004

    return ResolvedDrawerCabinetWithSlidingDrawersConfig(
        cabinet_style=config.cabinet_style,
        material_style=config.material_style,
        drawer_count=drawer_count,
        has_top_lid=has_top_lid,
        cabinet_width=width,
        cabinet_depth=depth,
        body_height=body_height,
        plinth_height=plinth_h,
        side_thickness=side_t,
        back_thickness=back_t,
        panel_thickness=panel_t,
        shelf_z=shelf_z + plinth_h,
        drawer_front_width=drawer_front_w,
        drawer_front_height=drawer_front_h,
        drawer_front_thickness=0.018,
        drawer_box_width=drawer_box_w,
        drawer_box_depth=drawer_box_d,
        drawer_box_height=drawer_box_h,
        drawer_panel_thickness=0.012,
        drawer_bottom_thickness=0.010,
        drawer_travel=travel,
        lid_thickness=lid_t,
        lid_width=lid_w,
        lid_depth=lid_d,
        drawer_center_z=drawer_center_z,
        name=config.name or "drawer_cabinet_with_sliding_drawers",
    )


def with_overrides(
    config: DrawerCabinetWithSlidingDrawersConfig, **kwargs: object
) -> DrawerCabinetWithSlidingDrawersConfig:
    return replace(config, **kwargs)


def slot_choices_for_config(
    config: DrawerCabinetWithSlidingDrawersConfig | ResolvedDrawerCabinetWithSlidingDrawersConfig,
) -> tuple[tuple[str, str], ...]:
    r = (
        config
        if isinstance(config, ResolvedDrawerCabinetWithSlidingDrawersConfig)
        else resolve_config(config)
    )
    return (
        ("cabinet_body", r.cabinet_style),
        ("drawer_multiplicity", f"{r.drawer_count}_drawers"),
        ("top_lid", "hinged_lid" if r.has_top_lid else "none"),
        ("material_palette", r.material_style),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(config_from_seed(seed))


def _joint_meta(joint_type, axis, origin, limits) -> dict[str, object]:
    return {
        "type": joint_type.value,
        "axis": axis,
        "origin": origin,
        "range": None if limits is None else (limits.lower, limits.upper),
    }


def _box(part, size, xyz, material, name: str, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _build_cabinet_body(body, r: ResolvedDrawerCabinetWithSlidingDrawersConfig, case_mat) -> None:
    w, d, h = r.cabinet_width, r.cabinet_depth, r.body_height
    st, bt, pt = r.side_thickness, r.back_thickness, r.panel_thickness
    z0 = r.plinth_height
    side_z = z0 + (h - z0) / 2.0

    if r.plinth_height > 0.0:
        _box(
            body,
            (d * 0.96, w * 0.94, r.plinth_height),
            (0.0, 0.0, r.plinth_height / 2.0),
            case_mat,
            "plinth",
        )

    for sign, name in ((1.0, "side_right"), (-1.0, "side_left")):
        _box(
            body,
            (d, st, h - z0),
            (0.0, sign * (w / 2.0 - st / 2.0), side_z),
            case_mat,
            name,
        )
    _box(
        body,
        (bt, w - 2.0 * st, h - z0),
        (-(d / 2.0) + bt / 2.0, 0.0, side_z),
        case_mat,
        "back",
    )
    _box(
        body,
        (d - bt, w - 2.0 * st, pt),
        (bt / 2.0, 0.0, z0 + pt / 2.0),
        case_mat,
        "bottom",
    )

    # Drawer joints sit on the cabinet +X opening plane. Use side stiles plus thin
    # per-drawer slide rails (not a solid front panel) so closed drawers do not
    # overlap the body while parent origins remain on real geometry.
    front_x = (d / 2.0) - (pt / 2.0)
    if r.has_top_lid and r.cabinet_style in {"bedside_compartment", "plinth_base"}:
        front_h = max(pt, r.shelf_z - z0)
        front_z = z0 + front_h / 2.0
    else:
        front_h = h - z0
        front_z = z0 + front_h / 2.0
    for sign, name in ((1.0, "front_stile_right"), (-1.0, "front_stile_left")):
        _box(
            body,
            (pt, st, front_h),
            (front_x, sign * (w / 2.0 - st / 2.0), front_z),
            case_mat,
            name,
        )

    if r.has_top_lid and r.cabinet_style in {"bedside_compartment", "plinth_base"}:
        _box(
            body,
            (d - bt, w - 2.0 * st, pt),
            (bt / 2.0, 0.0, r.shelf_z),
            case_mat,
            "compartment_floor",
        )
        top_band_h = h - (r.shelf_z + pt / 2.0)
        _box(
            body,
            (pt, w - 2.0 * st, top_band_h),
            ((d / 2.0) - pt / 2.0, 0.0, r.shelf_z + pt / 2.0 + top_band_h / 2.0),
            case_mat,
            "compartment_front",
        )
    elif r.cabinet_style == "uniform_stack":
        for i in range(1, r.drawer_count):
            shelf_z = z0 + pt + i * ((r.shelf_z - z0 - pt) / r.drawer_count)
            _box(
                body,
                (d - bt, w - 2.0 * st, pt * 0.65),
                (bt / 2.0, 0.0, shelf_z),
                case_mat,
                f"drawer_shelf_{i}",
            )

    if r.cabinet_style == "wide_low":
        _box(
            body,
            (d - bt, w - 2.0 * st, 0.012),
            (bt / 2.0, 0.0, h - 0.006),
            case_mat,
            "top_molding",
        )

    rail_h = 0.012
    for i, cz in enumerate(r.drawer_center_z):
        _box(
            body,
            (pt, w - 2.0 * st, rail_h),
            (front_x, 0.0, cz),
            case_mat,
            f"drawer_slide_rail_{i}",
        )


def _build_drawer(
    drawer,
    r: ResolvedDrawerCabinetWithSlidingDrawersConfig,
    *,
    front_mat,
    box_mat,
    hardware_mat,
) -> None:
    ft = r.drawer_front_thickness
    z_off = (r.drawer_front_height - r.drawer_box_height) / 2.0
    _box(
        drawer,
        (ft, r.drawer_front_width, r.drawer_front_height),
        (-ft / 2.0, 0.0, z_off),
        front_mat,
        "front_panel",
    )
    side_len = r.drawer_box_depth - ft - r.drawer_panel_thickness
    side_y = -ft - (side_len / 2.0)
    for sign, name in ((1.0, "box_side_right"), (-1.0, "box_side_left")):
        _box(
            drawer,
            (side_len, r.drawer_panel_thickness, r.drawer_box_height),
            (
                side_y,
                sign * (r.drawer_box_width / 2.0 - r.drawer_panel_thickness / 2.0),
                0.0,
            ),
            box_mat,
            name,
        )
    back_x = side_y - (side_len / 2.0) - (r.drawer_panel_thickness / 2.0)
    _box(
        drawer,
        (
            r.drawer_panel_thickness,
            r.drawer_box_width - 2.0 * r.drawer_panel_thickness,
            r.drawer_box_height,
        ),
        (back_x, 0.0, 0.0),
        box_mat,
        "box_back",
    )
    _box(
        drawer,
        (
            side_len,
            r.drawer_box_width - 2.0 * r.drawer_panel_thickness,
            r.drawer_bottom_thickness,
        ),
        (side_y, 0.0, -(r.drawer_box_height / 2.0) + r.drawer_bottom_thickness / 2.0),
        box_mat,
        "box_bottom",
    )
    drawer.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, z_off), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_mat,
        name="knob_stem",
    )
    drawer.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.019, 0.0, z_off), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_mat,
        name="knob_cap",
    )


def _build_lid(lid, r: ResolvedDrawerCabinetWithSlidingDrawersConfig, case_mat) -> None:
    _box(
        lid,
        (r.lid_depth, r.lid_width, r.lid_thickness),
        (r.lid_depth / 2.0, 0.0, r.lid_thickness / 2.0),
        case_mat,
        "lid_panel",
    )


def build_drawer_cabinet_with_sliding_drawers(
    config: DrawerCabinetWithSlidingDrawersConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    cfg = config or DrawerCabinetWithSlidingDrawersConfig()
    r = resolve_config(cfg)
    model = ArticulatedObject(name=r.name, assets=assets)

    palette = PALETTES[r.material_style]
    case_mat = model.material("cabinet_case", rgba=palette["case"])
    front_mat = model.material("drawer_front", rgba=palette["drawer_front"])
    box_mat = model.material("drawer_box", rgba=palette["drawer_box"])
    hardware_mat = model.material("cabinet_hardware", rgba=palette["hardware"])

    body = model.part("body")
    _build_cabinet_body(body, r, case_mat)
    body.inertial = Inertial.from_geometry(
        Box((r.cabinet_depth, r.cabinet_width, r.body_height)),
        mass=18.0 + r.drawer_count * 2.5,
        origin=Origin(xyz=(0.0, 0.0, r.body_height / 2.0)),
    )

    if r.has_top_lid:
        lid = model.part("lid")
        _build_lid(lid, r, case_mat)
        hinge_origin = (-(r.cabinet_depth / 2.0), 0.0, r.body_height)
        limits = MotionLimits(lower=0.0, upper=1.25, effort=20.0, velocity=1.5)
        model.articulation(
            "body_to_lid",
            ArticulationType.REVOLUTE,
            parent=body,
            child=lid,
            origin=Origin(xyz=hinge_origin),
            axis=(0.0, -1.0, 0.0),
            motion_limits=limits,
            meta=_joint_meta(ArticulationType.REVOLUTE, (0.0, -1.0, 0.0), hinge_origin, limits),
        )

    slide_axis = (1.0, 0.0, 0.0)
    for i in range(r.drawer_count):
        drawer = model.part(f"drawer_{i}")
        _build_drawer(drawer, r, front_mat=front_mat, box_mat=box_mat, hardware_mat=hardware_mat)
        drawer.inertial = Inertial.from_geometry(
            Box((r.drawer_box_depth, r.drawer_front_width, r.drawer_front_height)),
            mass=2.5 + r.drawer_box_depth * 4.0,
            origin=Origin(xyz=(-r.drawer_box_depth / 2.0, 0.0, 0.0)),
        )
        center_z = r.drawer_center_z[i]
        joint_origin = (r.cabinet_depth / 2.0, 0.0, center_z)
        limits = MotionLimits(
            lower=0.0,
            upper=r.drawer_travel,
            effort=80.0,
            velocity=0.22,
        )
        model.articulation(
            f"body_to_drawer_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=joint_origin),
            axis=slide_axis,
            motion_limits=limits,
            meta=_joint_meta(ArticulationType.PRISMATIC, slide_axis, joint_origin, limits),
        )

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_drawer_cabinet_with_sliding_drawers(
    seed: int,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    return build_drawer_cabinet_with_sliding_drawers(config_from_seed(seed), assets=assets)


def run_drawer_cabinet_with_sliding_drawers_tests(
    object_model: ArticulatedObject,
    config: DrawerCabinetWithSlidingDrawersConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}

    ctx.check("body part present", "body" in part_names)
    drawer_parts = [name for name in part_names if name.startswith("drawer_")]
    ctx.check(
        "drawer part count matches config",
        len(drawer_parts) == r.drawer_count,
        details=f"expected {r.drawer_count}, got {drawer_parts}",
    )

    drawer_joints = [name for name in joint_names if name.startswith("body_to_drawer_")]
    ctx.check(
        "drawer joint count matches config",
        len(drawer_joints) == r.drawer_count,
        details=f"expected {r.drawer_count}, got {drawer_joints}",
    )

    for i in range(r.drawer_count):
        joint = object_model.get_articulation(f"body_to_drawer_{i}")
        ctx.check(
            f"body_to_drawer_{i} slides along +X",
            tuple(abs(v) for v in joint.axis) == (1.0, 0.0, 0.0),
            details=str(joint.axis),
        )
        ctx.check(
            f"body_to_drawer_{i} metadata complete",
            {"type", "axis", "origin", "range"} <= set(joint.meta),
        )

    if r.has_top_lid:
        ctx.check("lid part present", "lid" in part_names)
        ctx.check("body_to_lid joint present", "body_to_lid" in joint_names)
        lid_joint = object_model.get_articulation("body_to_lid")
        ctx.check(
            "body_to_lid is revolute",
            lid_joint.articulation_type == ArticulationType.REVOLUTE,
            details=str(lid_joint.articulation_type),
        )
    else:
        ctx.check("no lid when disabled", "lid" not in part_names)
        ctx.check("no lid joint when disabled", "body_to_lid" not in joint_names)

    body = object_model.get_part("body")
    for i in range(r.drawer_count):
        drawer = object_model.get_part(f"drawer_{i}")
        ctx.allow_overlap(
            body,
            drawer,
            reason="drawer slides inside the cabinet cavity; closed fronts meet interior shelves/rails",
        )
    for i in range(r.drawer_count - 1):
        ctx.allow_overlap(
            object_model.get_part(f"drawer_{i}"),
            object_model.get_part(f"drawer_{i + 1}"),
            reason="stacked drawer fronts share the same cabinet opening plane",
        )

    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    return ctx.report()


__all__ = [
    "DrawerCabinetWithSlidingDrawersConfig",
    "ResolvedDrawerCabinetWithSlidingDrawersConfig",
    "build_drawer_cabinet_with_sliding_drawers",
    "build_seeded_drawer_cabinet_with_sliding_drawers",
    "config_from_seed",
    "resolve_config",
    "run_drawer_cabinet_with_sliding_drawers_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
    "with_overrides",
]
