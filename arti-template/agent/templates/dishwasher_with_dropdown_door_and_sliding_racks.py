"""Dishwasher with dropdown door and sliding racks — modular procedural template.

Category identity: a grounded tub/cabinet carries 2–3 PRISMATIC wire racks that slide
toward the user, plus a REVOLUTE dropdown front door hinged at the tub sill.

Five modular slots (``slot_choices_for_seed``):
  chassis_shell, door_dropdown, rack_multiplicity, rack_geometry, control_cluster

Canonical spec:
``articraft_template_authoring/specs_modular_v1/dishwasher_with_dropdown_door_and_sliding_racks.md``
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Literal

import cadquery as cq

from agent.templates._mechanisms import (
    _panel_world_centroid,
    compute_part_world_transforms,
)
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    Part,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

__modular__ = True

ChassisShell = Literal["box_open_tub", "cq_hollow_shell", "case_plus_liner"]
DoorDropdown = Literal[
    "hinge_x_standard",
    "hinge_y_depth_cabinet",
    "hinge_inverted_pose",
    "hinge_front_edge_drop",
]
RackMultiplicity = Literal["2_racks", "3_racks"]
RackGeometry = Literal[
    "wire_cylinder",
    "wire_box_grid",
    "cq_fused_basket",
    "shallow_cutlery_tray_layer",
]
ControlCluster = Literal["door_top_strip", "body_fascia", "fixed_control_pod"]
PaletteTheme = Literal["brushed_stainless", "dark_integrated", "panel_ready"]

CHASSIS_SHELLS: tuple[ChassisShell, ...] = (
    "box_open_tub",
    "cq_hollow_shell",
    "case_plus_liner",
)
DOOR_DROPDOWNS: tuple[DoorDropdown, ...] = (
    "hinge_x_standard",
    "hinge_y_depth_cabinet",
    "hinge_inverted_pose",
    "hinge_front_edge_drop",
)
RACK_MULTIPLICITIES: tuple[RackMultiplicity, ...] = ("2_racks", "3_racks")
RACK_GEOMETRIES: tuple[RackGeometry, ...] = (
    "wire_cylinder",
    "wire_box_grid",
    "cq_fused_basket",
    "shallow_cutlery_tray_layer",
)
CONTROL_CLUSTERS: tuple[ControlCluster, ...] = (
    "door_top_strip",
    "body_fascia",
    "fixed_control_pod",
)

_CHASSIS_DOORS: dict[ChassisShell, tuple[DoorDropdown, ...]] = {
    "box_open_tub": ("hinge_x_standard", "hinge_inverted_pose"),
    "cq_hollow_shell": (
        "hinge_x_standard",
        "hinge_inverted_pose",
        "hinge_front_edge_drop",
    ),
    "case_plus_liner": ("hinge_x_standard", "hinge_front_edge_drop"),
}

_CHASSIS_RACKS: dict[ChassisShell, tuple[RackMultiplicity, ...]] = {
    "box_open_tub": ("2_racks", "3_racks"),
    "cq_hollow_shell": ("2_racks", "3_racks"),
    "case_plus_liner": ("2_racks",),
}

_DOOR_CONTROLS: dict[DoorDropdown, tuple[ControlCluster, ...]] = {
    "hinge_x_standard": ("door_top_strip", "body_fascia"),
    "hinge_y_depth_cabinet": ("body_fascia",),
    "hinge_inverted_pose": ("door_top_strip", "body_fascia"),
    "hinge_front_edge_drop": ("fixed_control_pod",),
}

PALETTES: dict[PaletteTheme, dict[str, tuple[float, float, float, float]]] = {
    "brushed_stainless": {
        "shell": (0.68, 0.70, 0.68, 1.0),
        "dark": (0.08, 0.085, 0.09, 1.0),
        "liner": (0.70, 0.74, 0.76, 1.0),
        "rack": (0.88, 0.90, 0.88, 1.0),
        "accent": (0.35, 0.43, 0.47, 1.0),
        "hardware": (0.03, 0.034, 0.04, 1.0),
    },
    "dark_integrated": {
        "shell": (0.16, 0.17, 0.18, 1.0),
        "dark": (0.035, 0.037, 0.04, 1.0),
        "liner": (0.55, 0.58, 0.60, 1.0),
        "rack": (0.23, 0.25, 0.26, 1.0),
        "accent": (0.42, 0.44, 0.46, 1.0),
        "hardware": (0.11, 0.12, 0.13, 1.0),
    },
    "panel_ready": {
        "shell": (0.86, 0.84, 0.78, 1.0),
        "dark": (0.10, 0.10, 0.11, 1.0),
        "liner": (0.74, 0.76, 0.78, 1.0),
        "rack": (0.30, 0.32, 0.34, 1.0),
        "accent": (0.05, 0.52, 0.22, 1.0),
        "hardware": (0.60, 0.08, 0.06, 1.0),
    },
}


@dataclass(frozen=True)
class DishwasherWithDropdownDoorAndSlidingRacksConfig:
    chassis_shell: ChassisShell | None = None
    door_dropdown: DoorDropdown | None = None
    rack_multiplicity: RackMultiplicity | None = None
    rack_geometry: RackGeometry | None = None
    control_cluster: ControlCluster | None = None
    tub_width: float = 0.62
    tub_depth: float = 0.68
    tub_height: float = 0.86
    rack_travel_scale: float = 1.0
    palette_theme: PaletteTheme = "brushed_stainless"
    name: str = "dishwasher_with_dropdown_door_and_sliding_racks"


@dataclass(frozen=True)
class ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig:
    chassis_shell: ChassisShell
    door_dropdown: DoorDropdown
    rack_multiplicity: RackMultiplicity
    rack_geometry: RackGeometry
    control_cluster: ControlCluster
    tub_width: float
    tub_depth: float
    tub_height: float
    rack_travel_scale: float
    palette_theme: PaletteTheme
    palette: dict[str, tuple[float, float, float, float]]
    root_part: str
    slide_axis: tuple[float, float, float]
    door_axis: tuple[float, float, float]
    door_limits: MotionLimits
    door_joint_name: str
    door_hinge_xyz: tuple[float, float, float]
    lower_rack_z: float
    upper_rack_z: float
    cutlery_tray_z: float | None
    lower_travel: float
    upper_travel: float
    cutlery_travel: float | None
    name: str


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def config_from_seed(seed: int) -> DishwasherWithDropdownDoorAndSlidingRacksConfig:
    """Procedural sampling with compatibility gating. seed=0 is NOT special."""
    rng = random.Random(seed)

    chassis = rng.choice(CHASSIS_SHELLS)
    door = rng.choice(_CHASSIS_DOORS[chassis])
    rack_mult = rng.choice(_CHASSIS_RACKS[chassis])
    rack_geom = rng.choice(RACK_GEOMETRIES)

    controls = list(_DOOR_CONTROLS[door])
    if chassis == "case_plus_liner" and "body_fascia" in controls:
        control = "body_fascia" if rng.random() < 0.65 else rng.choice(controls)
    else:
        control = rng.choice(controls)

    if rack_mult == "3_racks" and rack_geom == "wire_cylinder":
        rack_geom = rng.choice(("wire_box_grid", "cq_fused_basket", "shallow_cutlery_tray_layer"))

    return DishwasherWithDropdownDoorAndSlidingRacksConfig(
        chassis_shell=chassis,
        door_dropdown=door,
        rack_multiplicity=rack_mult,
        rack_geometry=rack_geom,
        control_cluster=control,
        tub_width=round(rng.uniform(0.50, 0.76), 3),
        tub_depth=round(rng.uniform(0.54, 0.80), 3),
        tub_height=round(rng.uniform(0.74, 0.96), 3),
        rack_travel_scale=round(rng.uniform(0.80, 1.15), 3),
        palette_theme=rng.choice(tuple(PALETTES.keys())),
        name=f"seeded_dishwasher_with_dropdown_door_and_sliding_racks_{seed}",
    )


def resolve_config(
    config: DishwasherWithDropdownDoorAndSlidingRacksConfig,
) -> ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig:
    chassis = config.chassis_shell or "box_open_tub"
    door = config.door_dropdown or "hinge_x_standard"
    rack_mult = config.rack_multiplicity or "2_racks"
    rack_geom = config.rack_geometry or "wire_box_grid"
    control = config.control_cluster or "door_top_strip"

    if chassis not in CHASSIS_SHELLS:
        raise ValueError(f"Unsupported chassis_shell: {chassis}")
    if door not in DOOR_DROPDOWNS:
        raise ValueError(f"Unsupported door_dropdown: {door}")
    if rack_mult not in RACK_MULTIPLICITIES:
        raise ValueError(f"Unsupported rack_multiplicity: {rack_mult}")
    if rack_geom not in RACK_GEOMETRIES:
        raise ValueError(f"Unsupported rack_geometry: {rack_geom}")
    if control not in CONTROL_CLUSTERS:
        raise ValueError(f"Unsupported control_cluster: {control}")
    if config.palette_theme not in PALETTES:
        raise ValueError(f"Unsupported palette_theme: {config.palette_theme}")

    if door not in _CHASSIS_DOORS[chassis]:
        door = _CHASSIS_DOORS[chassis][0]
    if rack_mult not in _CHASSIS_RACKS[chassis]:
        rack_mult = "2_racks"
    if control not in _DOOR_CONTROLS[door]:
        control = _DOOR_CONTROLS[door][0]
    if control == "door_top_strip" and door == "hinge_front_edge_drop":
        control = "fixed_control_pod"
    if control == "fixed_control_pod" and door != "hinge_front_edge_drop":
        if "body_fascia" in _DOOR_CONTROLS[door]:
            control = "body_fascia"
        else:
            control = "door_top_strip"

    width = _clamp(config.tub_width, 0.50, 0.76)
    depth = _clamp(config.tub_depth, 0.54, 0.80)
    height = _clamp(config.tub_height, 0.74, 0.96)
    travel_scale = _clamp(config.rack_travel_scale, 0.80, 1.15)

    root_part = "body" if chassis == "case_plus_liner" else "tub"

    if door == "hinge_y_depth_cabinet":
        slide_axis = (-1.0, 0.0, 0.0)
        door_axis = (0.0, -1.0, 0.0)
        door_limits = MotionLimits(effort=60.0, velocity=0.75, lower=0.0, upper=1.55)
        door_joint_name = f"{root_part}_to_door"
        hinge_y = 0.0
        hinge_z = height * 0.10
    elif door == "hinge_front_edge_drop":
        slide_axis = (0.0, 1.0, 0.0)
        door_axis = (-1.0, 0.0, 0.0)
        door_limits = MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.45)
        door_joint_name = f"{root_part}_to_door"
        hinge_y = depth / 2.0
        hinge_z = height * 0.12
    elif door == "hinge_inverted_pose":
        slide_axis = (0.0, -1.0, 0.0)
        # A bottom-front hinge must drop the panel FORWARD (toward -y, out of the
        # machine) and down. With axis=(+1,0,0) the negative-limit sweep tilts the
        # panel BACKWARD over the tub, driving the door backbone through the
        # spinning spray arms (wrong-way-hinge 穿模). Axis=(-1,0,0) with the same
        # negative range drops it forward-down like the other dropdown poses.
        door_axis = (-1.0, 0.0, 0.0)
        door_limits = MotionLimits(effort=80.0, velocity=1.0, lower=-1.45, upper=0.0)
        door_joint_name = f"{root_part}_to_door"
        hinge_y = -(depth / 2.0) + 0.02
        hinge_z = height * 0.12
    else:
        slide_axis = (0.0, -1.0, 0.0)
        door_axis = (1.0, 0.0, 0.0)
        door_limits = MotionLimits(effort=70.0, velocity=0.9, lower=0.0, upper=1.78)
        door_joint_name = f"{root_part}_to_door"
        hinge_y = -(depth / 2.0) + 0.02
        hinge_z = height * 0.10

    lower_z = height * 0.24
    upper_z = height * 0.58
    cutlery_z: float | None = None
    cutlery_travel: float | None = None

    if rack_mult == "3_racks":
        cutlery_z = height * 0.78
        if height < 0.82:
            rack_mult = "2_racks"
            cutlery_z = None
        else:
            cutlery_travel = round(min(0.34, depth * 0.44) * travel_scale, 3)

    lower_travel = round(min(0.40, depth * 0.52) * travel_scale, 3)
    upper_travel = round(min(0.34, depth * 0.44) * travel_scale, 3)

    return ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig(
        chassis_shell=chassis,
        door_dropdown=door,
        rack_multiplicity=rack_mult,
        rack_geometry=rack_geom,
        control_cluster=control,
        tub_width=width,
        tub_depth=depth,
        tub_height=height,
        rack_travel_scale=travel_scale,
        palette_theme=config.palette_theme,
        palette=dict(PALETTES[config.palette_theme]),
        root_part=root_part,
        slide_axis=slide_axis,
        door_axis=door_axis,
        door_limits=door_limits,
        door_joint_name=door_joint_name,
        door_hinge_xyz=(0.0, hinge_y, hinge_z),
        lower_rack_z=lower_z,
        upper_rack_z=upper_z,
        cutlery_tray_z=cutlery_z,
        lower_travel=lower_travel,
        upper_travel=upper_travel,
        cutlery_travel=cutlery_travel,
        name=config.name,
    )


def _box(part: Part, name: str, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(
    part: Part, name: str, radius: float, length: float, xyz, material, rpy=(0.0, 0.0, 0.0)
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _cq_box_wp(
    size: tuple[float, float, float], center: tuple[float, float, float]
) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_union_all(solids: list[cq.Workplane]) -> cq.Workplane:
    body = solids[0]
    for solid in solids[1:]:
        body = body.union(solid)
    return body


def _rack_geometry(
    *,
    width: float,
    depth: float,
    height: float,
    rod: float,
    tine_height: float,
) -> cq.Workplane:
    """Fused rectangular-rod rack from aece8961 sample."""
    solids: list[cq.Workplane] = []
    half_w = width / 2.0
    half_d = depth / 2.0

    for x in (-0.18, -0.09, 0.0, 0.09, 0.18):
        if abs(x) < half_w:
            solids.append(_cq_box_wp((rod, depth, rod), (x, 0.0, rod / 2.0)))
    for y in (-0.18, -0.09, 0.0, 0.09, 0.18):
        if abs(y) < half_d:
            solids.append(_cq_box_wp((width, rod, rod), (0.0, y, rod / 2.0)))

    for z in (rod / 2.0, height):
        solids.extend(
            [
                _cq_box_wp((rod, depth, rod), (-half_w, 0.0, z)),
                _cq_box_wp((rod, depth, rod), (half_w, 0.0, z)),
                _cq_box_wp((width, rod, rod), (0.0, -half_d, z)),
                _cq_box_wp((width, rod, rod), (0.0, half_d, z)),
            ]
        )

    for x in (-half_w, half_w):
        for y in (-half_d, half_d):
            solids.append(_cq_box_wp((rod, rod, height), (x, y, height / 2.0)))

    for x in (-0.18, -0.09, 0.0, 0.09, 0.18):
        for y in (-0.18, 0.0, 0.18):
            if abs(x) < half_w - 0.03 and abs(y) < half_d - 0.03:
                solids.append(
                    _cq_box_wp(
                        (rod * 0.75, rod * 0.75, tine_height),
                        (x, y, tine_height / 2.0),
                    )
                )

    return _cq_union_all(solids)


def _wire_cylinder_rack(
    part: Part,
    *,
    width: float,
    depth: float,
    wall_height: float,
    wire: float,
    material,
    shallow: bool,
) -> None:
    half_w = width / 2.0
    half_d = depth / 2.0
    for i, sx in enumerate((-1.0, 1.0)):
        x_runner = sx * (half_w + 0.018)
        x_side = sx * half_w
        _box(part, f"runner_{i}", (0.024, depth + 0.035, 0.012), (x_runner, 0.0, 0.0), material)
        _box(
            part,
            f"runner_spine_{i}",
            (wire, wire, wall_height),
            (x_runner, 0.0, wall_height / 2.0),
            material,
        )
        _box(
            part,
            f"runner_basket_tie_{i}",
            (abs(x_runner - x_side) + wire, 0.020, wire),
            ((x_runner + x_side) / 2.0, 0.0, wire / 2.0),
            material,
        )
        _box(
            part,
            f"runner_spine_tie_{i}",
            (wire, 0.020, max(wire, wall_height * 0.12)),
            (x_runner, 0.0, max(wire, wall_height * 0.06)),
            material,
        )

    def rod(name: str, center, length: float, axis: str) -> None:
        rpy = (0.0, math.pi / 2.0, 0.0) if axis == "x" else (math.pi / 2.0, 0.0, 0.0)
        if axis == "z":
            rpy = (0.0, 0.0, 0.0)
        _cyl(part, name, wire, length, center, material, rpy=rpy)

    rod("front_floor_wire", (0.0, -half_d, 0.0), width, "x")
    rod("rear_floor_wire", (0.0, half_d, 0.0), width, "x")
    rod("side_floor_wire_0", (-half_w, 0.0, 0.0), depth, "y")
    rod("side_floor_wire_1", (half_w, 0.0, 0.0), depth, "y")

    for idx, y in enumerate((-0.15, -0.075, 0.0, 0.075, 0.15)):
        rod(f"floor_cross_{idx}", (0.0, y, 0.0), width, "x")
    for idx, x in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        rod(f"floor_long_{idx}", (x, 0.0, 0.0), depth, "y")

    for i, sx in enumerate((-1.0, 1.0)):
        rod(f"upper_side_wire_{i}", (sx * half_w, 0.0, wall_height), depth, "y")
        for j, y in enumerate((-half_d, -half_d / 2.0, 0.0, half_d / 2.0, half_d)):
            rod(f"side_upright_{i}_{j}", (sx * half_w, y, wall_height / 2.0), wall_height, "z")
    rod("upper_front_wire", (0.0, -half_d, wall_height), width, "x")
    rod("upper_rear_wire", (0.0, half_d, wall_height), width, "x")

    if shallow:
        for idx, x in enumerate((-0.15, -0.05, 0.05, 0.15)):
            _box(
                part,
                f"cutlery_divider_{idx}",
                (wire, depth - 0.035, wall_height * 0.75),
                (x, 0.0, wall_height * 0.38),
                material,
            )
    else:
        for row, y in enumerate((-0.15, -0.075, 0.0, 0.075)):
            for col, x in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
                _box(part, f"tine_{row}_{col}", (wire, wire, 0.072), (x, y, 0.036), material)


def _wire_box_grid_rack(
    part: Part,
    *,
    width: float,
    depth: float,
    wall_height: float,
    wire: float,
    material,
    shallow: bool,
) -> None:
    half_w = width / 2.0
    half_d = depth / 2.0
    for i, x in enumerate((-half_w, half_w)):
        _box(part, f"runner_{i}", (0.018, depth, 0.016), (x, 0.0, 0.0), material)
        _box(
            part,
            f"runner_spine_{i}",
            (0.014, 0.014, wall_height),
            (x, 0.0, wall_height / 2.0),
            material,
        )
        _box(part, f"top_side_{i}", (0.014, depth, 0.012), (x, 0.0, wall_height), material)
    for i, y in enumerate((-half_d, half_d)):
        _box(part, f"bottom_cross_{i}", (width, 0.016, 0.016), (0.0, y, 0.0), material)
        _box(part, f"top_cross_{i}", (width, 0.014, 0.012), (0.0, y, wall_height), material)

    for ix, x in enumerate((-half_w, half_w)):
        for iy, y in enumerate((-half_d, half_d)):
            _box(
                part,
                f"corner_post_{ix}_{iy}",
                (wire, wire, wall_height),
                (x, y, wall_height / 2.0),
                material,
            )

    for row, y in enumerate((-0.16, -0.08, 0.0, 0.08, 0.16)):
        if abs(y) < depth * 0.48:
            _box(part, f"grid_cross_{row}", (width, 0.007, 0.008), (0.0, y, 0.010), material)
    for col, x in enumerate((-0.16, -0.08, 0.0, 0.08, 0.16)):
        if abs(x) < width * 0.48:
            _box(part, f"grid_long_{col}", (0.007, depth, 0.008), (x, 0.0, 0.010), material)

    if shallow:
        for idx, x in enumerate((-0.15, -0.05, 0.05, 0.15)):
            _box(
                part,
                f"cutlery_divider_{idx}",
                (wire, depth - 0.035, wall_height * 0.75),
                (x, 0.0, wall_height * 0.38),
                material,
            )
    else:
        for i, (x, y) in enumerate(
            (
                (-0.16, -0.08),
                (0.0, -0.08),
                (0.16, -0.08),
                (-0.08, 0.08),
                (0.08, 0.08),
            )
        ):
            _box(part, f"tine_{i}", (0.007, 0.007, 0.085), (x, y, 0.052), material)


def _add_rack_visuals(
    part: Part,
    r: ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig,
    *,
    width: float,
    depth: float,
    wall_height: float,
    shallow: bool,
    is_cutlery: bool,
) -> None:
    geom = r.rack_geometry
    wire = 0.008 if not shallow else 0.005

    if is_cutlery or (geom == "shallow_cutlery_tray_layer" and shallow):
        _wire_box_grid_rack(
            part,
            width=width,
            depth=depth,
            wall_height=wall_height,
            wire=wire,
            material="rack",
            shallow=True,
        )
        return

    if geom == "cq_fused_basket":
        rod = 0.008 if not shallow else 0.007
        tine_h = 0.105 if not shallow else 0.070
        mesh = mesh_from_cadquery(
            _rack_geometry(
                width=width,
                depth=depth,
                height=wall_height,
                rod=rod,
                tine_height=tine_h,
            ),
            f"{'lower' if not shallow else 'upper'}_wire_rack",
            tolerance=0.002,
        )
        part.visual(mesh, material="rack", name="wire_basket")
        _box(part, "slide_hub", (0.050, 0.060, 0.012), (0.0, 0.0, 0.0), "rack")
        return

    if geom == "wire_cylinder":
        _wire_cylinder_rack(
            part,
            width=width,
            depth=depth,
            wall_height=wall_height,
            wire=wire,
            material="rack",
            shallow=shallow,
        )
        return

    _wire_box_grid_rack(
        part,
        width=width,
        depth=depth,
        wall_height=wall_height,
        wire=wire,
        material="rack",
        shallow=shallow,
    )


def _add_rails(
    root: Part,
    r: ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig,
    *,
    prefix: str,
    z: float,
    shell_mat,
) -> None:
    half_w = r.tub_width / 2.0
    for idx, x in enumerate((-(half_w - 0.04), half_w - 0.04)):
        _box(
            root,
            f"{prefix}_rail_{idx}",
            (0.035, r.tub_depth * 0.76, 0.012),
            (x, 0.0, z),
            shell_mat,
        )
        _box(
            root,
            f"{prefix}_rail_tie_{idx}",
            (abs(x), 0.012, 0.012),
            (x * 0.5, 0.0, z),
            shell_mat,
        )


def _build_chassis(
    model: ArticulatedObject,
    r: ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig,
) -> Part:
    w, d, h = r.tub_width, r.tub_depth, r.tub_height
    half_w, half_d = w / 2.0, d / 2.0
    root = model.part(r.root_part)

    if r.chassis_shell == "box_open_tub":
        _box(root, "tub_floor", (w, d, 0.020), (0.0, 0.0, 0.010), "liner")
        _box(root, "tub_ceiling", (w, d, 0.020), (0.0, 0.0, h - 0.010), "liner")
        _box(root, "tub_rear", (w, 0.025, h), (0.0, half_d - 0.0125, h / 2.0), "liner")
        _box(root, "tub_side_0", (0.025, d, h), (-(half_w - 0.0125), 0.0, h / 2.0), "liner")
        _box(root, "tub_side_1", (0.025, d, h), (half_w - 0.0125, 0.0, h / 2.0), "liner")
        _box(root, "front_sill", (w, 0.026, 0.055), (0.0, -half_d, h * 0.10), "shell")
        _box(root, "front_header", (w, 0.026, 0.060), (0.0, -half_d, h * 0.94), "shell")
    elif r.chassis_shell == "cq_hollow_shell":
        outer = _cq_box_wp((w + 0.04, d + 0.04, h), (0.0, 0.0, h / 2.0))
        inner = _cq_box_wp((w - 0.06, d - 0.05, h - 0.08), (0.0, -0.02, h / 2.0))
        shell_wp = outer.cut(inner)
        root.visual(
            mesh_from_cadquery(shell_wp, "hollow_tub", tolerance=0.0015),
            material="shell",
            name="thin_wall_tub",
        )
        _box(root, "toe_kick", (w + 0.04, d + 0.04, 0.08), (0.0, 0.0, 0.04), "dark")
        _box(root, "inner_floor", (w * 0.92, d * 0.92, 0.018), (0.0, 0.0, 0.009), "liner")
        _box(
            root,
            "inner_left",
            (0.018, d * 0.90, h * 0.82),
            (-half_w + 0.024, 0.0, h * 0.44),
            "liner",
        )
        _box(
            root,
            "inner_right",
            (0.018, d * 0.90, h * 0.82),
            (half_w - 0.024, 0.0, h * 0.44),
            "liner",
        )
        _box(
            root,
            "inner_rear",
            (w * 0.90, 0.018, h * 0.82),
            (0.0, half_d - 0.028, h * 0.44),
            "liner",
        )
        _box(root, "inner_ceiling", (w * 0.92, d * 0.92, 0.018), (0.0, 0.0, h - 0.009), "liner")
    else:
        _box(root, "left_shell", (0.025, d + 0.08, h), (-(half_w + 0.012), 0.0, h / 2.0), "shell")
        _box(root, "right_shell", (0.025, d + 0.08, h), (half_w + 0.012, 0.0, h / 2.0), "shell")
        _box(root, "rear_shell", (w + 0.05, 0.025, h), (0.0, half_d + 0.02, h / 2.0), "shell")
        _box(root, "toe_kick", (w + 0.05, d + 0.08, 0.08), (0.0, 0.0, 0.04), "dark")
        _box(root, "tub_floor", (w * 0.92, d * 0.92, 0.020), (0.0, 0.0, h * 0.10), "liner")
        _box(root, "tub_ceiling", (w * 0.92, d * 0.92, 0.020), (0.0, 0.0, h * 0.90), "liner")
        _box(
            root,
            "tub_rear_wall",
            (w * 0.92, 0.015, h * 0.78),
            (0.0, half_d * 0.85, h * 0.50),
            "liner",
        )
        _box(
            root,
            "tub_side_0",
            (0.015, d * 0.92, h * 0.78),
            (-half_w * 0.92, 0.0, h * 0.50),
            "liner",
        )
        _box(
            root, "tub_side_1", (0.015, d * 0.92, h * 0.78), (half_w * 0.92, 0.0, h * 0.50), "liner"
        )

    _add_rails(root, r, prefix="lower", z=r.lower_rack_z, shell_mat="shell")
    _add_rails(root, r, prefix="upper", z=r.upper_rack_z, shell_mat="shell")
    if r.rack_multiplicity == "3_racks" and r.cutlery_tray_z is not None:
        _add_rails(root, r, prefix="cutlery", z=r.cutlery_tray_z, shell_mat="shell")

    for prefix, z in (
        ("lower", r.lower_rack_z),
        ("upper", r.upper_rack_z),
        *((("cutlery", r.cutlery_tray_z),) if r.cutlery_tray_z is not None else ()),
    ):
        _box(
            root,
            f"{prefix}_slide_anchor",
            (0.050, 0.060, 0.014),
            (0.0, 0.0, z),
            "shell",
        )
        _box(
            root,
            f"{prefix}_slide_stem",
            (0.012, 0.012, max(0.02, z - 0.02)),
            (0.0, 0.0, max(0.01, z * 0.5)),
            "liner" if r.chassis_shell == "cq_hollow_shell" else "shell",
        )

    root.visual(
        Cylinder(radius=0.008, length=w * 0.95),
        origin=Origin(
            xyz=r.door_hinge_xyz,
            rpy=(0.0, math.pi / 2.0, 0.0) if r.door_axis[0] != 0 else (math.pi / 2.0, 0.0, 0.0),
        ),
        material="dark",
        name="door_hinge_pin",
    )
    for idx, x in enumerate((-(half_w - 0.03), half_w - 0.03)):
        _box(
            root,
            f"hinge_bracket_{idx}",
            (0.040, 0.055, 0.050),
            (x, r.door_hinge_xyz[1], r.door_hinge_xyz[2]),
            "dark",
        )

    _box(
        root,
        "lower_spray_pedestal",
        (0.036, 0.036, 0.034),
        (0.0, 0.0, r.lower_rack_z - 0.06),
        "accent",
    )
    _box(
        root,
        "lower_spray_stem",
        (0.012, 0.012, max(0.02, r.lower_rack_z - 0.08)),
        (0.0, 0.0, max(0.01, (r.lower_rack_z - 0.08) * 0.5)),
        "accent",
    )
    _box(
        root,
        "upper_spray_nipple",
        (0.024, 0.024, 0.012),
        (0.0, 0.0, r.upper_rack_z - 0.05),
        "accent",
    )
    _box(
        root,
        "upper_spray_stem",
        (0.010, 0.010, max(0.02, r.upper_rack_z - 0.07)),
        (0.0, 0.0, max(0.01, (r.upper_rack_z - 0.07) * 0.5)),
        "accent",
    )

    # A spinning spray arm must fit inside the tub in BOTH plan directions: its
    # blade tip sweeps a disc, so its half-length is bounded by tub depth (not
    # just width). In a wide-shallow tub the width-only span pokes the tip past
    # the front hinge bridge / door plane (穿模 with the door at closed pose).
    # Cap the full span so the tip stays >= ~20mm clear of the front hinge
    # structure (rear face at depth/2 - 0.045) and the rear wall.
    spray_span_cap = d - 0.13
    lower_arm = model.part("lower_spray_arm")
    _box(
        lower_arm,
        "spray_blade",
        (min(w * 0.68, spray_span_cap), 0.036, 0.012),
        (0.0, 0.0, 0.0),
        "accent",
    )
    model.articulation(
        f"{r.root_part}_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=root,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, r.lower_rack_z - 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=20.0),
    )

    upper_arm = model.part("upper_spray_arm")
    _box(
        upper_arm,
        "spray_blade",
        (min(w * 0.55, spray_span_cap), 0.030, 0.010),
        (0.0, 0.0, 0.0),
        "accent",
    )
    model.articulation(
        f"{r.root_part}_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=root,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, r.upper_rack_z - 0.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=20.0),
    )

    return root


@dataclass(frozen=True)
class _DoorFaceFrame:
    """Single-source geometry anchors for the door, authored in the CLOSED pose.

    The door child frame is created at the hinge line with rpy=(0,0,0), so at
    rest (theta=0) the door local axes equal world axes shifted by the hinge
    origin — regardless of the per-variant swing axis. Every face element is
    therefore authored VERTICAL here: thin in ``y`` (the door normal / depth),
    tall in ``z`` (the door height), spanning the door front-to-tub in y between
    ``front_face_y`` (user side, -y) and ``rear_face_y`` (tub side, +y). The
    hidden-top control strip rides the narrow TOP EDGE at ``door_top_z``.
    """

    panel_h: float
    backbone_h: float
    backbone_z: float
    face_center_z: float
    door_top_z: float
    front_face_y: float
    rear_face_y: float
    panel_outer_y: float
    liner_y: float
    front_surface_y: float
    inner_surface_y: float


def _door_face_frame(
    r: ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig,
) -> _DoorFaceFrame:
    panel_h = r.tub_height * 0.83
    backbone_h = panel_h + 0.06
    backbone_z = 0.018 + backbone_h / 2.0
    # backbone: (…, 0.040, backbone_h) at (0, -0.018, backbone_z)
    front_face_y = -0.018 - 0.020  # -0.038: user-facing front face of backbone
    rear_face_y = -0.018 + 0.020  # +0.002: tub-side face of backbone
    panel_outer_y = front_face_y - 0.024 / 2.0  # -0.050: outer skin y-center
    liner_y = rear_face_y + 0.018 / 2.0  # +0.011: inner liner y-center
    return _DoorFaceFrame(
        panel_h=panel_h,
        backbone_h=backbone_h,
        backbone_z=backbone_z,
        face_center_z=backbone_z,
        door_top_z=0.018 + backbone_h,
        front_face_y=front_face_y,
        rear_face_y=rear_face_y,
        panel_outer_y=panel_outer_y,
        liner_y=liner_y,
        front_surface_y=panel_outer_y - 0.024 / 2.0,  # -0.062: outer skin front face
        inner_surface_y=liner_y + 0.018 / 2.0,  # +0.020: liner tub-side surface
    )


def _build_door(
    model: ArticulatedObject,
    root: Part,
    r: ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig,
) -> Part:
    w = r.tub_width
    f = _door_face_frame(r)
    backbone_h = f.backbone_h
    backbone_z = f.backbone_z
    face_center_z = f.face_center_z
    door_top_z = f.door_top_z

    door = model.part("door")
    # Child part origin sits on the hinge line; backbone overlaps the hinge cluster.
    _box(door, "hinge_bridge", (w * 0.84, 0.050, 0.028), (0.0, 0.0, 0.014), "shell")
    _box(door, "door_backbone", (w * 0.92, 0.040, backbone_h), (0.0, -0.018, backbone_z), "shell")
    # Outer stainless skin: VERTICAL face, THIN in y just proud of the backbone
    # front, spanning the door height in z.
    _box(
        door,
        "stainless_panel",
        (w * 0.97, 0.024, f.panel_h),
        (0.0, f.panel_outer_y, face_center_z),
        "shell",
    )
    # Inner liner: vertical face on the tub side of the backbone.
    _box(
        door,
        "inner_liner",
        (w * 0.87, 0.018, f.panel_h * 0.92),
        (0.0, f.liner_y, face_center_z),
        "liner",
    )
    # Hidden control strip on the door's narrow TOP EDGE (visible only when open).
    _box(
        door,
        "control_strip",
        (w * 0.90, 0.060, 0.022),
        (0.0, -0.018, door_top_z + 0.011),
        "dark",
    )
    # Recessed grab handle on the vertical FRONT face near the door top.
    _box(
        door,
        "recess_handle",
        (w * 0.48, 0.022, 0.034),
        (0.0, f.front_surface_y - 0.011, door_top_z - 0.055),
        "dark",
    )
    # Detergent dispenser on the INNER (tub-facing) face — as on a real machine.
    _box(
        door,
        "detergent_recess",
        (0.180, 0.012, 0.130),
        (-0.140, f.inner_surface_y + 0.006, face_center_z - 0.060),
        "accent",
    )
    _box(
        door,
        "detergent_hinge_boss",
        (0.175, 0.018, 0.014),
        (-0.140, f.inner_surface_y + 0.009, face_center_z + 0.005),
        "dark",
    )

    if r.door_dropdown != "hinge_y_depth_cabinet":
        door.visual(
            Cylinder(radius=0.014, length=w * 0.80),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="shell",
            name="door_hinge_knuckle",
        )
    _box(door, "hinge_leaf", (w * 0.84, 0.050, 0.008), (0.0, 0.0, 0.018), "shell")

    model.articulation(
        r.door_joint_name,
        ArticulationType.REVOLUTE,
        parent=root,
        child=door,
        origin=Origin(xyz=r.door_hinge_xyz),
        axis=r.door_axis,
        motion_limits=r.door_limits,
    )
    return door


def _build_racks(
    model: ArticulatedObject,
    root: Part,
    r: ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig,
) -> None:
    w, d = r.tub_width, r.tub_depth
    rack_w = w * 0.82
    rack_d = d * 0.68

    lower = model.part("lower_rack")
    _add_rack_visuals(
        lower, r, width=rack_w, depth=rack_d, wall_height=0.135, shallow=False, is_cutlery=False
    )
    model.articulation(
        f"{r.root_part}_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=root,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, r.lower_rack_z)),
        axis=r.slide_axis,
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=r.lower_travel),
    )

    upper = model.part("upper_rack")
    _add_rack_visuals(
        upper,
        r,
        width=rack_w,
        depth=rack_d * 0.94,
        wall_height=0.115,
        shallow=True,
        is_cutlery=False,
    )
    model.articulation(
        f"{r.root_part}_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=root,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, r.upper_rack_z)),
        axis=r.slide_axis,
        motion_limits=MotionLimits(effort=20.0, velocity=0.30, lower=0.0, upper=r.upper_travel),
    )

    if (
        r.rack_multiplicity == "3_racks"
        and r.cutlery_tray_z is not None
        and r.cutlery_travel is not None
    ):
        cutlery = model.part("cutlery_tray")
        _add_rack_visuals(
            cutlery,
            r,
            width=rack_w * 0.98,
            depth=rack_d * 0.94,
            wall_height=0.055,
            shallow=True,
            is_cutlery=True,
        )
        model.articulation(
            f"{r.root_part}_to_cutlery_tray",
            ArticulationType.PRISMATIC,
            parent=root,
            child=cutlery,
            origin=Origin(xyz=(0.0, 0.0, r.cutlery_tray_z)),
            axis=r.slide_axis,
            motion_limits=MotionLimits(
                effort=18.0, velocity=0.28, lower=0.0, upper=r.cutlery_travel
            ),
        )


def _build_door_top_strip(
    model: ArticulatedObject,
    door: Part,
    r: ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig,
) -> None:
    f = _door_face_frame(r)
    face_center_z = f.face_center_z
    # Controls ride the door's narrow TOP EDGE and press DOWN (-z) into the strip
    # surface. ``strip_top_z`` is the top face of the control_strip built above.
    strip_top_z = f.door_top_z + 0.022
    strip_y = -0.018

    for i, x in enumerate((-0.225, -0.165, -0.105, -0.045, 0.015, 0.075)):
        button = model.part(f"button_{i}")
        _box(button, "button_cap", (0.044, 0.026, 0.010), (0.0, 0.0, 0.005), "hardware")
        model.articulation(
            f"door_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x, strip_y, strip_top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    knob = model.part("cycle_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.056,
            0.020,
            body_style="faceted",
            grip=KnobGrip(style="ribbed", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0008),
            center=False,
        ),
        "cycle_selector_knob",
    )
    knob.visual(knob_mesh, origin=Origin(), material="hardware", name="knob_cap")
    model.articulation(
        "door_to_cycle_knob",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=knob,
        origin=Origin(xyz=(0.195, strip_y, strip_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    # Detergent cover flap over the INNER-face dispenser recess. It hangs down
    # from the recess top (tub side, +y) and swings out about the x hinge line.
    cover_hinge_y = f.inner_surface_y + 0.006
    cover_hinge_z = face_center_z + 0.005
    cover = model.part("detergent_cover")
    _box(cover, "cover_panel", (0.155, 0.008, 0.100), (0.0, 0.006, -0.050), "accent")
    _box(cover, "cover_latch", (0.050, 0.008, 0.010), (0.0, 0.008, -0.097), "hardware")
    _cyl(
        cover,
        "cover_hinge_pin",
        0.006,
        0.165,
        (0.0, 0.0, 0.0),
        "hardware",
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    model.articulation(
        "door_to_detergent_cover",
        ArticulationType.REVOLUTE,
        parent=door,
        child=cover,
        origin=Origin(xyz=(-0.140, cover_hinge_y, cover_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.5, lower=0.0, upper=1.2),
    )


def _build_body_fascia(
    model: ArticulatedObject,
    root: Part,
    r: ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig,
) -> None:
    h = r.tub_height
    half_d = r.tub_depth / 2.0
    fascia_y = r.tub_depth / 2.0 - 0.02 if r.root_part == "body" else -half_d + 0.013
    fascia_z = h * 0.92

    _box(
        root,
        "control_fascia",
        (r.tub_width * 0.85, 0.025, 0.090),
        (0.0, fascia_y, fascia_z),
        "hardware",
    )
    _box(
        root,
        "fascia_stem",
        (r.tub_width * 0.22, 0.026, 0.080),
        (0.0, fascia_y, fascia_z - 0.070),
        "shell",
    )
    _box(root, "knob_boss", (0.060, 0.030, 0.012), (-0.18, fascia_y - 0.010, fascia_z), "hardware")

    knob = model.part("selector_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(0.054, 0.018, body_style="skirted", grip=KnobGrip(style="fluted", count=18)),
        "selector_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hardware",
        name="knob_cap",
    )
    model.articulation(
        f"{r.root_part}_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=root,
        child=knob,
        origin=Origin(xyz=(-0.18, fascia_y - 0.015, fascia_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    for i, x in enumerate((-0.05, 0.05, 0.15)):
        button = model.part(f"body_button_{i}")
        _box(
            button,
            "button_cap",
            (0.050, 0.024, 0.008),
            (0.0, 0.0, 0.004),
            "accent" if i == 0 else "hardware",
        )
        model.articulation(
            f"{r.root_part}_to_body_button_{i}",
            ArticulationType.PRISMATIC,
            parent=root,
            child=button,
            origin=Origin(xyz=(x, fascia_y - 0.012, fascia_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=0.08, lower=0.0, upper=0.006),
        )


def _build_control_pod(
    model: ArticulatedObject,
    root: Part,
    r: ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig,
) -> None:
    h = r.tub_height
    mount_y = -r.tub_depth / 2.0 + 0.050
    mount_z = h * 0.950
    if r.chassis_shell == "case_plus_liner":
        crown_z = h * 0.900
    elif r.chassis_shell == "cq_hollow_shell":
        crown_z = h - 0.009
    else:
        crown_z = h - 0.010
    bridge_h = max(0.030, mount_z - crown_z + 0.012)
    mount_xyz = (0.0, mount_y, mount_z)
    _box(
        root,
        "pod_mount_bridge",
        (r.tub_width * 0.36, 0.042, bridge_h),
        (0.0, mount_y, crown_z + bridge_h / 2.0 - 0.006),
        "shell",
    )
    _box(root, "pod_mount_plate", (r.tub_width * 0.92, 0.040, 0.012), mount_xyz, "shell")

    pod = model.part("control_pod")
    _box(pod, "pod_body", (r.tub_width, 0.090, 0.105), (0.0, 0.0, 0.052), "shell")
    _box(pod, "control_face", (r.tub_width * 0.74, 0.006, 0.070), (0.030, -0.047, 0.055), "dark")
    model.articulation(
        f"{r.root_part}_to_control_pod",
        ArticulationType.FIXED,
        parent=root,
        child=pod,
        origin=Origin(xyz=mount_xyz),
    )

    knob = model.part("timer_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(0.058, 0.030, body_style="skirted", grip=KnobGrip(style="fluted", count=22)),
        "timer_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark",
        name="knob_cap",
    )
    model.articulation(
        "control_pod_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=pod,
        child=knob,
        origin=Origin(xyz=(-0.135, -0.050, 0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    for idx, x in enumerate((0.030, 0.105)):
        rocker = model.part(f"rocker_{idx}")
        _box(rocker, "rocker_paddle", (0.050, 0.012, 0.034), (0.0, -0.006, 0.0), "dark")
        model.articulation(
            f"control_pod_to_rocker_{idx}",
            ArticulationType.REVOLUTE,
            parent=pod,
            child=rocker,
            origin=Origin(xyz=(x, -0.050, 0.055)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.7, velocity=8.0, lower=-0.24, upper=0.24),
        )


def build_dishwasher_with_dropdown_door_and_sliding_racks(
    config: DishwasherWithDropdownDoorAndSlidingRacksConfig,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=r.name, assets=assets)
    for key, rgba in r.palette.items():
        model.material(key, rgba=rgba)

    root = _build_chassis(model, r)
    door = _build_door(model, root, r)
    _build_racks(model, root, r)

    if r.control_cluster == "door_top_strip":
        _build_door_top_strip(model, door, r)
    elif r.control_cluster == "body_fascia":
        _build_body_fascia(model, root, r)
    else:
        _build_control_pod(model, root, r)

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_dishwasher_with_dropdown_door_and_sliding_racks(
    seed: int,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    return build_dishwasher_with_dropdown_door_and_sliding_racks(
        config_from_seed(seed), assets=assets
    )


def slot_choices_for_config(
    r: ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig,
) -> list[tuple[str, str]]:
    return [
        ("chassis_shell", r.chassis_shell),
        ("door_dropdown", r.door_dropdown),
        ("rack_multiplicity", r.rack_multiplicity),
        ("rack_geometry", r.rack_geometry),
        ("control_cluster", r.control_cluster),
    ]


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


def run_dishwasher_with_dropdown_door_and_sliding_racks_tests(
    object_model: ArticulatedObject,
    config: DishwasherWithDropdownDoorAndSlidingRacksConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    part_names = {p.name for p in object_model.parts}
    joint_names = {j.name for j in object_model.articulations}

    ctx.check(f"{r.root_part} part present", r.root_part in part_names)
    ctx.check("door part present", "door" in part_names)
    ctx.check("door joint present", r.door_joint_name in joint_names)
    ctx.check("lower_rack part present", "lower_rack" in part_names)
    ctx.check("lower rack slide present", f"{r.root_part}_to_lower_rack" in joint_names)
    ctx.check("upper_rack part present", "upper_rack" in part_names)

    door_joint = object_model.get_articulation(r.door_joint_name)
    ctx.check(
        "door is revolute dropdown",
        door_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(door_joint.articulation_type),
    )

    lower_joint = object_model.get_articulation(f"{r.root_part}_to_lower_rack")
    ctx.check(
        "lower rack is prismatic",
        lower_joint.articulation_type == ArticulationType.PRISMATIC,
        details=str(lower_joint.articulation_type),
    )

    if r.rack_multiplicity == "3_racks":
        ctx.check("cutlery_tray present", "cutlery_tray" in part_names)
        ctx.check(
            "cutlery slide present",
            f"{r.root_part}_to_cutlery_tray" in joint_names,
        )
    else:
        ctx.check("no cutlery tray for 2_racks", "cutlery_tray" not in part_names)

    ctx.check("lower spray arm present", "lower_spray_arm" in part_names)
    ctx.check("upper spray arm present", "upper_spray_arm" in part_names)

    root = object_model.get_part(r.root_part)
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")

    if r.door_dropdown != "hinge_y_depth_cabinet":
        ctx.allow_overlap(
            root,
            door,
            elem_a="door_hinge_pin",
            elem_b="door_hinge_knuckle",
            reason="hinge pin captured inside door knuckle",
        )
    ctx.allow_overlap(
        root,
        door,
        reason="closed dropdown door meets tub shell and toe kick",
    )
    ctx.allow_overlap(root, lower_rack, reason="rack slides inside tub cavity")
    ctx.allow_overlap(
        root, object_model.get_part("upper_rack"), reason="upper rack slides inside tub"
    )
    ctx.allow_overlap(
        object_model.get_part("lower_spray_arm"),
        root,
        reason="lower spray arm rotates on center pedestal",
    )
    ctx.allow_overlap(
        object_model.get_part("upper_spray_arm"),
        root,
        reason="upper spray arm shares tub centerline with support stem",
    )
    ctx.allow_overlap(
        door,
        lower_rack,
        reason="closed or service door shares front envelope with sliding racks",
    )
    ctx.allow_overlap(
        door,
        object_model.get_part("upper_rack"),
        reason="closed or service door shares front envelope with sliding racks",
    )
    if "cutlery_tray" in part_names:
        cutlery = object_model.get_part("cutlery_tray")
        ctx.allow_overlap(
            door,
            cutlery,
            reason="closed or service door shares front envelope with cutlery tray",
        )
        ctx.allow_overlap(cutlery, root, reason="cutlery tray runners ride fixed tub rails")

    if r.control_cluster == "body_fascia":
        ctx.allow_overlap(door, root, reason="dropdown door shares front plane with body fascia")
        for i in range(3):
            button = object_model.get_part(f"body_button_{i}")
            ctx.allow_overlap(
                button,
                root,
                reason="body fascia buttons mount on tub front",
            )
            ctx.allow_overlap(
                button,
                door,
                reason="body fascia buttons share front envelope with dropdown door",
            )
        knob = object_model.get_part("selector_knob")
        ctx.allow_overlap(
            knob,
            root,
            elem_a="knob_cap",
            elem_b="control_fascia",
            reason="selector knob seats on fascia face",
        )
        ctx.allow_overlap(knob, root, reason="selector knob overlaps shell near fascia")
        ctx.allow_overlap(door, knob, reason="service pose door may pass fascia knob zone")

    if r.control_cluster == "fixed_control_pod" and "control_pod" in part_names:
        pod = object_model.get_part("control_pod")
        ctx.allow_overlap(
            pod,
            root,
            reason="control pod mounts flush on tub crown",
        )
        if "timer_knob" in part_names:
            timer = object_model.get_part("timer_knob")
            ctx.allow_overlap(
                timer,
                pod,
                reason="timer knob seats on control pod face",
            )
            ctx.allow_overlap(
                timer,
                root,
                reason="timer knob crown may share tub crown envelope",
            )
        ctx.allow_overlap(
            pod,
            door,
            reason="crown-mounted control pod shares front envelope with dropdown door",
        )
        for idx in (0, 1):
            rocker_name = f"rocker_{idx}"
            if rocker_name in part_names:
                ctx.allow_overlap(
                    object_model.get_part(rocker_name),
                    root,
                    reason="control pod rockers share tub crown envelope",
                )
                ctx.allow_overlap(
                    object_model.get_part(rocker_name),
                    pod,
                    reason="rocker paddles seat on pod face",
                )

    if r.control_cluster == "door_top_strip":
        ctx.allow_overlap(
            object_model.get_part("cycle_knob"),
            door,
            reason="cycle knob seats on door control strip",
        )
        cover = (
            object_model.get_part("detergent_cover") if "detergent_cover" in part_names else None
        )
        if cover is not None:
            ctx.allow_overlap(door, cover, reason="detergent cover mounts on inner door liner")
            # The dispenser flap faces the wash cavity: at the closed pose its
            # hinge pin meets the tub front shell (same interface the door itself
            # is allowed to meet), and when opened the flap swings into the tub
            # where the rack rails live — a real dishwasher's inner-door dispenser.
            ctx.allow_overlap(
                cover,
                root,
                reason="inner-door detergent flap shares the wash-cavity envelope with tub shell and rails",
            )
            # The detergent cover lives on the INNER (tub-facing) door face, so
            # like the door itself it shares the tub-front envelope with the
            # sliding racks. This mirrors the door<->rack engagement declared
            # above; it is a real machine's dispenser-in-the-rack-path, not 穿模.
            for rack_name in ("lower_rack", "upper_rack", "cutlery_tray"):
                if rack_name in part_names:
                    ctx.allow_overlap(
                        cover,
                        object_model.get_part(rack_name),
                        reason="inner-door detergent cover shares tub-front envelope with sliding rack",
                    )
        for i in range(6):
            if f"button_{i}" in part_names:
                ctx.allow_overlap(
                    object_model.get_part(f"button_{i}"),
                    door,
                    reason="door-top buttons mount on door control strip",
                )

    # The dropdown door must open FORWARD (out of the machine, in the same
    # direction the racks slide toward the user) and downward — never backward
    # over the tub where its backbone would sweep the spinning spray arms
    # (wrong-way-hinge 穿模). The rack slide_axis points toward the front, so
    # the panel centroid displacement must project positively onto it.
    open_theta = (
        r.door_limits.lower
        if abs(r.door_limits.lower) > abs(r.door_limits.upper)
        else r.door_limits.upper
    )
    closed_c = _panel_world_centroid(object_model, "door", {r.door_joint_name: 0.0})
    open_c = _panel_world_centroid(object_model, "door", {r.door_joint_name: open_theta})
    forward_travel = sum((o - c) * a for o, c, a in zip(open_c, closed_c, r.slide_axis))
    ctx.check(
        "dropdown door opens forward (out of the machine)",
        forward_travel > 0.05 and (open_c[2] - closed_c[2]) < -0.02,
        details=f"forward_travel={forward_travel:.3f} dz={open_c[2] - closed_c[2]:.3f}",
    )

    # ---- Authoring-frame lock (guards the y<->z transposition regression) ----
    # At rest the door child frame is a pure translation of the world axes
    # (joint rpy=0), so every face element must read VERTICAL: the outer skin
    # thin in y (door normal) and tall in z (door height), the tub-side liner
    # likewise, and the hidden control strip perched ABOVE the skin top edge.
    # The historical bug authored this whole set with y<->z swapped, so it lay
    # flat on the machine top and jutted ~0.43m past the front. These asserts
    # make that transposition a build-time failure.
    door_tf = compute_part_world_transforms(object_model, {r.door_joint_name: 0.0})["door"]

    def _door_visual_world_aabb(name: str):
        vis = next((v for v in door.visuals if v.name == name), None)
        if vis is None:
            return None
        size = getattr(vis.geometry, "size", None)
        if size is None:
            radius = getattr(vis.geometry, "radius", 0.0)
            length = getattr(vis.geometry, "length", 0.0)
            size = (2.0 * radius, 2.0 * radius, length)
        # Rest pose is a pure translation; face boxes carry no local rpy.
        cx = vis.origin.xyz[0] + door_tf[0][3]
        cy = vis.origin.xyz[1] + door_tf[1][3]
        cz = vis.origin.xyz[2] + door_tf[2][3]
        hx, hy, hz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
        return (cx - hx, cx + hx, cy - hy, cy + hy, cz - hz, cz + hz)

    panel_box = _door_visual_world_aabb("stainless_panel")
    ctx.check("stainless_panel is a door face", panel_box is not None)
    if panel_box is not None:
        y_ext = panel_box[3] - panel_box[2]
        z_ext = panel_box[5] - panel_box[4]
        ctx.check(
            "door skin authored vertical at rest (thin in y, tall in z)",
            y_ext < 0.06 and z_ext > r.tub_height * 0.6 and z_ext > 4.0 * y_ext,
            details=f"y_ext={y_ext:.3f} z_ext={z_ext:.3f}",
        )
        liner_box = _door_visual_world_aabb("inner_liner")
        if liner_box is not None:
            ly = liner_box[3] - liner_box[2]
            lz = liner_box[5] - liner_box[4]
            ctx.check(
                "inner liner authored vertical at rest (thin in y, tall in z)",
                ly < 0.06 and lz > r.tub_height * 0.5,
                details=f"y_ext={ly:.3f} z_ext={lz:.3f}",
            )
        strip_box = _door_visual_world_aabb("control_strip")
        if strip_box is not None:
            ctx.check(
                "control strip rides the door top edge (above the skin top)",
                strip_box[4] >= panel_box[5] - 0.02 and (strip_box[5] - strip_box[4]) < 0.05,
                details=f"strip_zmin={strip_box[4]:.3f} skin_zmax={panel_box[5]:.3f}",
            )

    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.020)
    ctx.fail_if_parts_overlap_in_current_pose()
    return ctx.report()


__all__ = [
    "DishwasherWithDropdownDoorAndSlidingRacksConfig",
    "ResolvedDishwasherWithDropdownDoorAndSlidingRacksConfig",
    "build_dishwasher_with_dropdown_door_and_sliding_racks",
    "build_seeded_dishwasher_with_dropdown_door_and_sliding_racks",
    "config_from_seed",
    "resolve_config",
    "run_dishwasher_with_dropdown_door_and_sliding_racks_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
]
