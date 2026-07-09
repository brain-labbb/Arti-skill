from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireSidewall,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# Blue mesh office armchair baseline.
# The rolling caster base, gas-lift layout, tall mesh back, and two real
# prismatic T-armrest height sliders are bound to Other/armchair.

N_SPOKES = 4

SPOKE_LEN = 0.265
HUB_RADIUS = 0.050
HUB_HEIGHT = 0.052
HUB_Z = 0.072
BASE_SWIVEL_Z = HUB_Z - HUB_HEIGHT / 2.0
BEARING_HEIGHT = 0.018
BEARING_RADIUS = HUB_RADIUS * 1.24

WHEEL_RADIUS = 0.026
WHEEL_WIDTH = 0.013
WHEEL_GAP = 0.054
CASTER_OUT = 0.040
CASTER_R = SPOKE_LEN + CASTER_OUT

SLEEVE_RADIUS = 0.031
ROD_RADIUS = 0.019
SLEEVE_BOTTOM_Z = 0.050
SLEEVE_TOP_Z = 0.200
ROD_TOP_Z = 0.430

SEAT_W = 0.455
SEAT_D = 0.425
SEAT_THICK = 0.080
SEAT_BOTTOM_Z = ROD_TOP_Z
SEAT_TOP_Z = SEAT_BOTTOM_Z + SEAT_THICK

BACK_W = 0.420
BACK_H = 0.540
BACK_Y = -0.220
BACK_BOTTOM_Z = SEAT_TOP_Z + 0.015
BACK_TOP_Z = BACK_BOTTOM_Z + BACK_H
MESH_EDGE_X = BACK_W * 0.5 - 0.017
MESH_BOTTOM_Z = BACK_BOTTOM_Z + 0.024
MESH_TOP_Z = BACK_TOP_Z - 0.024
MESH_H_COUNT = 19
MESH_V_COUNT = 13
MESH_CORNER_Z = 0.082
BACK_HINGE_Z = BACK_BOTTOM_Z - 0.050

ARM_X = SEAT_W * 0.570
ARM_Y = 0.000
ARM_SLEEVE_BOTTOM_Z = SEAT_BOTTOM_Z - 0.005
ARM_SLEEVE_TOP_Z = SEAT_TOP_Z + 0.095
ARM_PAD_REST_Z = ARM_SLEEVE_TOP_Z + 0.118
ARM_TRAVEL = 0.080

JOINT_Z = HUB_Z


def bz(world_z: float) -> float:
    return world_z - BASE_SWIVEL_Z


def sz(world_z: float) -> float:
    return world_z - JOINT_Z


def _back_y_for_world_z(world_z: float) -> float:
    t = min(1.0, max(0.0, (world_z - BACK_BOTTOM_Z) / BACK_H))
    recline = BACK_Y - 0.060 * t
    lumbar_forward = 0.026 * math.exp(-((t - 0.30) / 0.22) ** 2)
    return recline + lumbar_forward


def _back_y_for_local_z(local_z: float) -> float:
    return _back_y_for_world_z(local_z + JOINT_Z)


def _back_hinge_y() -> float:
    return -0.245


def hsz(world_z: float) -> float:
    return world_z - BACK_HINGE_Z


def hby(world_z: float, offset: float = 0.0) -> float:
    return _back_y_for_world_z(world_z) + offset - _back_hinge_y()


def _mesh_half_width_for_world_z(world_z: float) -> float:
    if world_z < MESH_BOTTOM_Z + MESH_CORNER_Z:
        s = max(0.0, (world_z - MESH_BOTTOM_Z) / MESH_CORNER_Z)
        return MESH_EDGE_X - 0.055 * (1.0 - s) ** 2
    if world_z > MESH_TOP_Z - MESH_CORNER_Z:
        s = max(0.0, (MESH_TOP_Z - world_z) / MESH_CORNER_Z)
        return MESH_EDGE_X - 0.055 * (1.0 - s) ** 2
    return MESH_EDGE_X


def _mesh_y_offset_for_clearance(u: float, world_z: float, base_offset: float) -> float:
    return base_offset


def _mesh_point(u: float, world_z: float, y_offset: float = 0.0) -> tuple[float, float, float]:
    return (
        u * _mesh_half_width_for_world_z(world_z),
        hby(world_z, _mesh_y_offset_for_clearance(u, world_z, y_offset)),
        hsz(world_z),
    )


def _rounded_box_mesh(width: float, depth: float, height: float, name: str, *, bottom_at_zero: bool = False) -> object:
    solid = cq.Workplane("XY").box(
        width,
        depth,
        height,
        centered=(True, True, not bottom_at_zero),
    )
    return mesh_from_cadquery(solid, name)


def _annular_tube_mesh(outer_radius: float, inner_radius: float, height: float, name: str) -> object:
    solid = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    return mesh_from_cadquery(solid, name)


def _rounded_rect_path(width: float, depth: float, z: float, radius: float, segments: int = 8) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    hw = width * 0.5
    hd = depth * 0.5
    corners = [
        (hw - radius, hd - radius, 0.0),
        (-hw + radius, hd - radius, math.pi / 2.0),
        (-hw + radius, -hd + radius, math.pi),
        (hw - radius, -hd + radius, 3.0 * math.pi / 2.0),
    ]
    for cx, cy, start in corners:
        for i in range(segments + 1):
            a = start + (math.pi / 2.0) * i / segments
            pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a), z))
    return pts


def _seat_outline_xy(width: float, depth: float, radius: float, segments: int = 10) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    hw = width * 0.5
    hd = depth * 0.5
    corners = [
        (hw - radius, hd - radius, 0.0),
        (-hw + radius, hd - radius, math.pi / 2.0),
        (-hw + radius, -hd + radius, math.pi),
        (hw - radius, -hd + radius, 3.0 * math.pi / 2.0),
    ]
    for cx, cy, start in corners:
        for i in range(segments + 1):
            a = start + (math.pi / 2.0) * i / segments
            x = cx + radius * math.cos(a)
            y = cy + radius * math.sin(a)
            front = max(0.0, y / hd)
            rear = max(0.0, -y / hd)
            x *= 1.0 - 0.050 * rear
            y += 0.015 * front * front * (1.0 - min(1.0, abs(x) / hw) ** 2)
            pts.append((x, y))
    return pts


def _seat_outline_wire(width: float, depth: float, z: float, radius: float, name: str) -> cq.Wire:
    return cq.Workplane("XY").workplane(offset=z).polyline(_seat_outline_xy(width, depth, radius)).close().val()


def _seat_cushion_mesh() -> object:
    sections = [
        (0.000, SEAT_W * 0.88, SEAT_D * 0.84, 0.060),
        (0.014, SEAT_W * 0.980, SEAT_D * 0.955, 0.078),
        (0.052, SEAT_W * 1.020, SEAT_D * 1.015, 0.104),
        (0.075, SEAT_W * 0.990, SEAT_D * 0.975, 0.096),
        (0.083, SEAT_W * 0.900, SEAT_D * 0.865, 0.072),
    ]
    wires = [_seat_outline_wire(width, depth, z, radius, f"seat_cushion_section_{i}") for i, (z, width, depth, radius) in enumerate(sections)]
    return mesh_from_cadquery(cq.Solid.makeLoft(wires, ruled=False), "blue_sculpted_task_chair_cushion")


def _seat_piping_mesh() -> object:
    pts = [(x, y, SEAT_THICK * 0.64) for x, y in _seat_outline_xy(SEAT_W * 1.020, SEAT_D * 1.015, 0.104, segments=12)]
    geom = tube_from_spline_points(pts, radius=0.0052, closed_spline=True, radial_segments=10, cap_ends=False)
    return mesh_from_geometry(geom, "raised_seat_edge_wire")


def _backrest_frame_mesh() -> object:
    pts: list[tuple[float, float, float]] = []
    hw = BACK_W * 0.5
    radius = 0.065
    z0 = BACK_BOTTOM_Z
    z1 = BACK_TOP_Z
    corners = [
        (hw - radius, z1 - radius, 0.0),
        (-hw + radius, z1 - radius, math.pi / 2.0),
        (-hw + radius, z0 + radius, math.pi),
        (hw - radius, z0 + radius, 3.0 * math.pi / 2.0),
    ]
    for cx, cz, start in corners:
        for i in range(10):
            a = start + (math.pi / 2.0) * i / 9.0
            world_z = cz + radius * math.sin(a)
            pts.append((cx + radius * math.cos(a), hby(world_z), hsz(world_z)))
    geom = tube_from_spline_points(pts, radius=0.017, closed_spline=True, radial_segments=14, cap_ends=False)
    return mesh_from_geometry(geom, "teal_backrest_frame")


def _line_mesh(points: list[tuple[float, float, float]], radius: float, name: str) -> object:
    return mesh_from_geometry(
        tube_from_spline_points(points, radius=radius, radial_segments=7, cap_ends=True),
        name,
    )


def _arm_pad_mesh() -> object:
    return _rounded_box_mesh(0.072, 0.248, 0.028, "wide_t_arm_pad")


def _spoke_mesh(direction: tuple[float, float]) -> object:
    dx, dy = direction
    r0 = HUB_RADIUS - 0.012
    p0 = (dx * r0, dy * r0, bz(HUB_Z + 0.006))
    p1 = (dx * CASTER_R * 0.42, dy * CASTER_R * 0.42, bz(HUB_Z - 0.004))
    p2 = (dx * CASTER_R * 0.78, dy * CASTER_R * 0.78, bz(0.062))
    p3 = (dx * CASTER_R, dy * CASTER_R, bz(0.056))
    inner = tube_from_spline_points([p0, p1], radius=0.022, samples_per_segment=8, radial_segments=14, cap_ends=True)
    outer = tube_from_spline_points([p1, p2, p3], radius=0.016, samples_per_segment=10, radial_segments=12, cap_ends=True)
    return mesh_from_geometry(inner.merge(outer), "spoke")


def _caster_yoke_mesh(cx: float, cy: float, ax: float, ay: float) -> object:
    prong_half = WHEEL_GAP / 2.0 + WHEEL_WIDTH / 2.0 + 0.004
    prong_z = bz(WHEEL_RADIUS - 0.002)
    stem = tube_from_spline_points(
        [(cx, cy, bz(0.058)), (cx, cy, bz(WHEEL_RADIUS + 0.018))],
        radius=0.013,
        samples_per_segment=4,
        radial_segments=12,
        cap_ends=True,
    )
    tine_l = tube_from_spline_points(
        [
            (cx, cy, bz(WHEEL_RADIUS + 0.015)),
            (cx - ax * prong_half * 0.6, cy - ay * prong_half * 0.6, bz(WHEEL_RADIUS + 0.008)),
            (cx - ax * prong_half, cy - ay * prong_half, prong_z),
        ],
        radius=0.007,
        samples_per_segment=6,
        radial_segments=10,
        cap_ends=True,
    )
    tine_r = tube_from_spline_points(
        [
            (cx, cy, bz(WHEEL_RADIUS + 0.015)),
            (cx + ax * prong_half * 0.6, cy + ay * prong_half * 0.6, bz(WHEEL_RADIUS + 0.008)),
            (cx + ax * prong_half, cy + ay * prong_half, prong_z),
        ],
        radius=0.007,
        samples_per_segment=6,
        radial_segments=10,
        cap_ends=True,
    )
    return mesh_from_geometry(stem.merge(tine_l).merge(tine_r), "caster_yoke")


def _axle_mesh(a: tuple[float, float, float], b: tuple[float, float, float]) -> object:
    return mesh_from_geometry(tube_from_spline_points([a, b], radius=0.0090, samples_per_segment=4, radial_segments=12), "caster_axle")


def _twin_wheel_mesh() -> object:
    tire = TireGeometry(
        outer_radius=WHEEL_RADIUS,
        width=WHEEL_WIDTH,
        inner_radius=0.008,
        carcass=TireCarcass(belt_width_ratio=0.65, sidewall_bulge=0.14),
        sidewall=TireSidewall(style="rounded", bulge=0.12),
        center=True,
    )
    return mesh_from_geometry(tire, "caster_wheel")


def _gas_lift_column_cq() -> object:
    sl_bot = SLEEVE_BOTTOM_Z - HUB_Z
    sl_top = SLEEVE_TOP_Z - HUB_Z
    rd_top = ROD_TOP_Z - HUB_Z
    step_h = 0.018
    profile = [
        (0.0, sl_bot),
        (SLEEVE_RADIUS, sl_bot),
        (SLEEVE_RADIUS, sl_top - step_h),
        (ROD_RADIUS + 0.004, sl_top),
        (ROD_RADIUS, sl_top + step_h),
        (ROD_RADIUS, rd_top),
        (0.0, rd_top),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=48), "gas_lift_column")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blue_mesh_office_task_chair")

    model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("column_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("seat_blue_fabric", rgba=(0.05, 0.46, 0.74, 1.0))
    model.material("seat_piping_blue", rgba=(0.02, 0.30, 0.52, 1.0))
    model.material("back_teal_frame", rgba=(0.00, 0.24, 0.34, 1.0))
    model.material("back_blue_mesh", rgba=(0.02, 0.38, 0.58, 0.55))
    model.material("mesh_thread", rgba=(0.00, 0.60, 0.82, 1.0))
    model.material("arm_black", rgba=(0.035, 0.035, 0.040, 1.0))
    model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    model.material("steel", rgba=(0.30, 0.31, 0.33, 1.0))

    bearing = model.part("stationary_bearing")
    bearing.visual(
        Cylinder(radius=BEARING_RADIUS, length=BEARING_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_SWIVEL_Z - BEARING_HEIGHT / 2.0)),
        material="steel",
        name="bearing_washer",
    )

    base = model.part("caster_base")
    base.visual(Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT), origin=Origin(xyz=(0.0, 0.0, bz(HUB_Z))), material="base_black", name="hub")
    base.visual(
        Cylinder(radius=HUB_RADIUS * 0.72, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, bz(HUB_Z + HUB_HEIGHT / 2.0 + 0.006))),
        material="base_black",
        name="hub_collar",
    )

    spoke_dirs: list[tuple[float, float]] = []
    for i in range(N_SPOKES):
        a = 2.0 * math.pi * i / N_SPOKES + math.pi / 2.0
        d = (math.cos(a), math.sin(a))
        spoke_dirs.append(d)
        base.visual(_spoke_mesh(d), material="base_black", name=f"spoke_{i}")

    for i, d in enumerate(spoke_dirs):
        cx = d[0] * CASTER_R
        cy = d[1] * CASTER_R
        ax, ay = -d[1], d[0]
        base.visual(_caster_yoke_mesh(cx, cy, ax, ay), material="base_black", name=f"caster_yoke_{i}")
        half = WHEEL_GAP / 2.0 + 0.010
        base.visual(
            _axle_mesh(
                (cx - ax * half, cy - ay * half, bz(WHEEL_RADIUS)),
                (cx + ax * half, cy + ay * half, bz(WHEEL_RADIUS)),
            ),
            material="steel",
            name=f"caster_axle_{i}",
        )

    for i, d in enumerate(spoke_dirs):
        caster_cx = d[0] * CASTER_R
        caster_cy = d[1] * CASTER_R
        ax, ay = -d[1], d[0]
        for j, sgn in enumerate((-1.0, 1.0)):
            wx = caster_cx + ax * (WHEEL_GAP / 2.0) * sgn
            wy = caster_cy + ay * (WHEEL_GAP / 2.0) * sgn
            wheel = model.part(f"caster_wheel_{i}_{j}")
            wheel.visual(_twin_wheel_mesh(), origin=Origin(rpy=(0.0, 0.0, math.atan2(ay, ax))), material="rubber", name="tire")
            model.articulation(
                f"caster_roll_{i}_{j}",
                ArticulationType.CONTINUOUS,
                parent=base,
                child=wheel,
                origin=Origin(xyz=(wx, wy, bz(WHEEL_RADIUS))),
                axis=(ax, ay, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=20.0),
            )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=bearing,
        child=base,
        origin=Origin(xyz=(0.0, 0.0, BASE_SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )

    swivel = model.part("column_seat")
    swivel.visual(_gas_lift_column_cq(), origin=Origin(xyz=(0.0, 0.0, 0.0)), material="column_black", name="gas_lift_column")
    swivel.visual(
        Cylinder(radius=0.062, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, sz(SEAT_BOTTOM_Z - 0.006))),
        material="steel",
        name="seat_mount",
    )
    swivel.visual(
        Box((0.196, 0.168, 0.052)),
        origin=Origin(xyz=(0.0, -0.018, sz(SEAT_BOTTOM_Z - 0.028))),
        material="arm_black",
        name="tilt_mechanism_housing",
    )
    swivel.visual(
        Box((0.250, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, -0.108, sz(SEAT_BOTTOM_Z - 0.016))),
        material="arm_black",
        name="rear_hinge_lower_crossmember",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        swivel.visual(
            Box((0.044, 0.038, 0.034)),
            origin=Origin(xyz=(sign * 0.090, -0.112, sz(SEAT_BOTTOM_Z - 0.018))),
            material="arm_black",
            name=f"rear_hinge_{side_name}_rail_foot",
        )
    swivel.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(0.110, 0.040, sz(SEAT_BOTTOM_Z - 0.046)), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="arm_black",
        name="right_tilt_lever",
    )
    swivel.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(xyz=(-0.100, -0.056, sz(SEAT_BOTTOM_Z - 0.040)), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="arm_black",
        name="tension_knob",
    )
    swivel.visual(_seat_cushion_mesh(), origin=Origin(xyz=(0.0, 0.0, sz(SEAT_BOTTOM_Z))), material="seat_blue_fabric", name="cushion")
    swivel.visual(_seat_piping_mesh(), origin=Origin(xyz=(0.0, 0.0, sz(SEAT_BOTTOM_Z))), material="steel", name="seat_piping")

    swivel.visual(
        Box((0.285, 0.044, 0.042)),
        origin=Origin(xyz=(0.0, -SEAT_D * 0.5 + 0.010, sz(SEAT_BOTTOM_Z + 0.012))),
        material="arm_black",
        name="rear_seat_hinge_mount_block",
    )
    swivel.visual(
        Cylinder(radius=0.0125, length=0.310),
        origin=Origin(xyz=(0.0, _back_hinge_y(), sz(BACK_HINGE_Z)), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="arm_black",
        name="back_hinge_axle",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        swivel.visual(
            Box((0.026, 0.048, 0.044)),
            origin=Origin(xyz=(sign * 0.142, _back_hinge_y(), sz(BACK_HINGE_Z + 0.006))),
            material="arm_black",
            name=f"back_hinge_{side_name}_cheek",
        )
    swivel.visual(
        Box((0.240, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, _back_hinge_y() + 0.022, sz(BACK_HINGE_Z - 0.045))),
        material="arm_black",
        name="back_hinge_base_plate",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        swivel.visual(
            _line_mesh(
                [
                    (sign * 0.090, -0.112, sz(SEAT_BOTTOM_Z - 0.020)),
                    (sign * 0.112, _back_hinge_y() + 0.020, sz(BACK_HINGE_Z - 0.045)),
                ],
                0.009,
                f"back_hinge_{side_name}_support_rail",
            ),
            material="arm_black",
            name=f"back_hinge_{side_name}_support_rail",
        )
        swivel.visual(
            _line_mesh(
                [
                    (sign * 0.118, -SEAT_D * 0.5 + 0.002, sz(SEAT_BOTTOM_Z + 0.018)),
                    (sign * 0.140, _back_hinge_y() + 0.004, sz(BACK_HINGE_Z + 0.004)),
                ],
                0.010,
                f"back_hinge_{side_name}_upper_link",
            ),
            material="arm_black",
            name=f"back_hinge_{side_name}_upper_link",
        )

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        x = sign * ARM_X
        swivel.visual(
            _annular_tube_mesh(0.024, 0.0165, ARM_SLEEVE_TOP_Z - ARM_SLEEVE_BOTTOM_Z, f"{side_name}_hollow_armrest_sleeve"),
            origin=Origin(xyz=(x, ARM_Y, sz(ARM_SLEEVE_BOTTOM_Z))),
            material="arm_black",
            name=f"{side_name}_armrest_outer_hollow_sleeve",
        )
        swivel.visual(
            _annular_tube_mesh(0.029, 0.0165, 0.014, f"{side_name}_armrest_top_rim"),
            origin=Origin(xyz=(x, ARM_Y, sz(ARM_SLEEVE_TOP_Z - 0.010))),
            material="arm_black",
            name=f"{side_name}_armrest_top_open_rim",
        )
        swivel.visual(
            _annular_tube_mesh(0.027, 0.0165, 0.014, f"{side_name}_armrest_bottom_rim"),
            origin=Origin(xyz=(x, ARM_Y, sz(ARM_SLEEVE_BOTTOM_Z + 0.002))),
            material="arm_black",
            name=f"{side_name}_armrest_bottom_open_rim",
        )
        swivel.visual(
            Box((0.060, 0.144, 0.032)),
            origin=Origin(xyz=(sign * (SEAT_W * 0.45), ARM_Y, sz(SEAT_BOTTOM_Z + 0.020))),
            material="arm_black",
            name=f"{side_name}_armrest_seat_bracket",
        )
        swivel.visual(
            _line_mesh(
                [
                    (sign * (SEAT_W * 0.44), ARM_Y - 0.055, sz(SEAT_BOTTOM_Z + 0.015)),
                    (x, ARM_Y - 0.020, sz(ARM_SLEEVE_BOTTOM_Z + 0.040)),
                ],
                0.008,
                f"{side_name}_armrest_rear_triangular_brace",
            ),
            material="arm_black",
            name=f"{side_name}_armrest_rear_triangular_brace",
        )
        swivel.visual(
            _line_mesh(
                [
                    (sign * (SEAT_W * 0.44), ARM_Y + 0.055, sz(SEAT_BOTTOM_Z + 0.015)),
                    (x, ARM_Y + 0.020, sz(ARM_SLEEVE_BOTTOM_Z + 0.040)),
                ],
                0.008,
                f"{side_name}_armrest_front_triangular_brace",
            ),
            material="arm_black",
            name=f"{side_name}_armrest_front_triangular_brace",
        )
        swivel.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(x + sign * 0.023, ARM_Y - 0.010, sz(ARM_SLEEVE_TOP_Z - 0.050)), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="steel",
            name=f"{side_name}_armrest_height_release_button",
        )

    back_frame = model.part("backrest_frame")
    back_frame.visual(_backrest_frame_mesh(), material="back_teal_frame", name="backrest_frame")
    back_frame.visual(
        Cylinder(radius=0.018, length=0.088),
        origin=Origin(xyz=(-0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="arm_black",
        name="back_hinge_barrel_left",
    )
    back_frame.visual(
        Cylinder(radius=0.018, length=0.088),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="arm_black",
        name="back_hinge_barrel_right",
    )
    for side_name, x in (("left", -0.060), ("right", 0.060)):
        back_frame.visual(
            Box((0.034, 0.032, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.024)),
            material="arm_black",
            name=f"back_hinge_{side_name}_upper_clevis",
        )
        back_frame.visual(
            _line_mesh(
                [
                    (x, 0.0, 0.021),
                    (x * 1.14, hby(BACK_BOTTOM_Z + 0.030, 0.004), hsz(BACK_BOTTOM_Z + 0.030)),
                ],
                0.008,
                f"back_hinge_{side_name}_strap",
            ),
            material="arm_black",
            name=f"back_hinge_{side_name}_strap",
        )
    back_frame.visual(
        Box((0.186, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, hby(BACK_BOTTOM_Z + 0.010, 0.008), hsz(BACK_BOTTOM_Z + 0.010))),
        material="arm_black",
        name="back_hinge_lower_crossbar",
    )
    for side, u in (("left", -1.0), ("right", 1.0)):
        pts = []
        for j in range(19):
            world_z = MESH_BOTTOM_Z + j * ((MESH_TOP_Z - MESH_BOTTOM_Z) / 18.0)
            pts.append(_mesh_point(u, world_z, 0.006))
        back_frame.visual(
            _line_mesh(pts, 0.0048, f"mesh_{side}_tension_edge"),
            material="mesh_thread",
            name=f"mesh_{side}_tension_edge",
        )
        inner_pts = []
        for j in range(19):
            world_z = MESH_BOTTOM_Z + j * ((MESH_TOP_Z - MESH_BOTTOM_Z) / 18.0)
            inner_pts.append(_mesh_point(u * 0.925, world_z, 0.008))
        back_frame.visual(
            _line_mesh(inner_pts, 0.0022, f"mesh_{side}_inner_seam"),
            material="mesh_thread",
            name=f"mesh_{side}_inner_seam",
        )
    for tag, world_z in (("bottom", MESH_BOTTOM_Z), ("top", MESH_TOP_Z)):
        back_frame.visual(
            _line_mesh(
                [_mesh_point(-1.0 + k * 0.1, world_z, 0.006) for k in range(21)],
                0.0044,
                f"mesh_{tag}_tension_edge",
            ),
            material="mesh_thread",
            name=f"mesh_{tag}_tension_edge",
        )
    for tag, world_z in (("bottom", MESH_BOTTOM_Z + 0.018), ("top", MESH_TOP_Z - 0.018)):
        back_frame.visual(
            _line_mesh(
                [_mesh_point(-0.92 + k * (1.84 / 18.0), world_z, 0.008) for k in range(19)],
                0.0020,
                f"mesh_{tag}_inner_seam",
            ),
            material="mesh_thread",
            name=f"mesh_{tag}_inner_seam",
        )

    for i in range(MESH_H_COUNT):
        world_z = MESH_BOTTOM_Z + i * ((MESH_TOP_Z - MESH_BOTTOM_Z) / (MESH_H_COUNT - 1))
        back_frame.visual(
            _line_mesh([_mesh_point(-1.0 + k * 0.1, world_z, 0.010) for k in range(21)], 0.00135, f"mesh_h_{i}"),
            material="mesh_thread",
            name=f"mesh_h_{i}",
        )

    for i in range(MESH_V_COUNT):
        u = -1.0 + i * (2.0 / (MESH_V_COUNT - 1))
        pts = []
        for j in range(17):
            world_z = MESH_BOTTOM_Z + j * ((MESH_TOP_Z - MESH_BOTTOM_Z) / 16.0)
            weave_sag = 0.006 * math.sin(math.pi * j / 16.0) * (1.0 - min(1.0, abs(u)))
            half_w = _mesh_half_width_for_world_z(world_z)
            y_offset = _mesh_y_offset_for_clearance(u, world_z, 0.012)
            pts.append((u * half_w + weave_sag, hby(world_z, y_offset), hsz(world_z)))
        back_frame.visual(
            _line_mesh(pts, 0.00125, f"mesh_v_{i}"),
            material="mesh_thread",
            name=f"mesh_v_{i}",
        )
    lumbar_z = BACK_BOTTOM_Z + 0.155
    back_frame.visual(
        _rounded_box_mesh(0.265, 0.020, 0.070, "subtle_lumbar_pad"),
        origin=Origin(xyz=(0.0, hby(lumbar_z, 0.012), hsz(lumbar_z))),
        material="seat_piping_blue",
        name="lumbar_pad",
    )
    model.articulation(
        "backrest_recline_hinge",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=back_frame,
        origin=Origin(xyz=(0.0, _back_hinge_y(), sz(BACK_HINGE_Z))),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=math.radians(-10.0), upper=math.radians(18.0), effort=18.0, velocity=1.0),
    )

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        slider = model.part(f"{side_name}_armrest_slider")
        slider.visual(
            Cylinder(radius=0.0125, length=0.210),
            origin=Origin(xyz=(0.0, 0.0, -0.032)),
            material="arm_black",
            name="sliding_upper_post",
        )
        slider.visual(
            _annular_tube_mesh(0.020, 0.0118, 0.034, "sliding_low_friction_bushing"),
            origin=Origin(xyz=(0.0, 0.0, -0.120)),
            material="steel",
            name="sliding_low_friction_bushing",
        )
        slider.visual(
            _annular_tube_mesh(0.022, 0.0118, 0.052, "under_pad_hollow_socket"),
            origin=Origin(xyz=(0.0, 0.0, ARM_PAD_REST_Z - ARM_SLEEVE_TOP_Z - 0.054 - 0.036)),
            material="arm_black",
            name="under_pad_hollow_socket",
        )
        slider.visual(
            _arm_pad_mesh(),
            origin=Origin(xyz=(0.0, 0.0, ARM_PAD_REST_Z - ARM_SLEEVE_TOP_Z - 0.036)),
            material="arm_black",
            name="rounded_t_pad",
        )
        model.articulation(
            f"{side_name}_armrest_height",
            ArticulationType.PRISMATIC,
            parent=swivel,
            child=slider,
            origin=Origin(xyz=(sign * ARM_X, ARM_Y, sz(ARM_SLEEVE_TOP_Z))),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(lower=0.0, upper=ARM_TRAVEL, effort=20.0, velocity=0.20),
        )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, bz(JOINT_Z))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bearing = object_model.get_part("stationary_bearing")
    base = object_model.get_part("caster_base")
    swivel = object_model.get_part("column_seat")
    back_frame = object_model.get_part("backrest_frame")
    back_panel = back_frame
    base_swivel = object_model.get_articulation("base_swivel")
    seat_swivel = object_model.get_articulation("seat_swivel")
    back_hinge = object_model.get_articulation("backrest_recline_hinge")

    ctx.expect_contact(bearing, base, elem_a="bearing_washer", elem_b="hub", name="base hub rides on visible bearing washer")
    ctx.allow_overlap(base, swivel, elem_a="hub", elem_b="gas_lift_column", reason="Gas-lift column seats into the base hub bore.")
    ctx.allow_overlap(base, swivel, elem_a="hub_collar", elem_b="gas_lift_column", reason="Gas-lift column passes through the hub collar.")
    ctx.allow_overlap(swivel, back_frame, elem_a="back_hinge_axle", elem_b="back_hinge_barrel_left", reason="Left backrest hinge barrel rotates around the fixed cross axle.")
    ctx.allow_overlap(swivel, back_frame, elem_a="back_hinge_axle", elem_b="back_hinge_barrel_right", reason="Right backrest hinge barrel rotates around the fixed cross axle.")

    for i in range(N_SPOKES):
        for j in range(2):
            wheel = object_model.get_part(f"caster_wheel_{i}_{j}")
            ctx.allow_overlap(base, wheel, elem_a=f"caster_axle_{i}", elem_b="tire", reason="Caster wheel is captured on its axle.")
            for tine_suffix in ("__component_002", "__component_003"):
                ctx.allow_overlap(base, wheel, elem_a=f"caster_yoke_{i}{tine_suffix}", elem_b="tire", reason="Caster yoke fork prong embraces the wheel.")

    seat_aabb = ctx.part_element_world_aabb(swivel, elem="cushion")
    assert seat_aabb is not None
    seat_top = seat_aabb[1][2]
    sx = seat_aabb[1][0] - seat_aabb[0][0]
    sy = seat_aabb[1][1] - seat_aabb[0][1]
    ctx.check("blue cushion at office-chair height", 0.49 <= seat_top <= 0.54, details=f"seat_top={seat_top:.3f}")
    ctx.check("seat is wide rounded-rectangle", sx > 0.43 and sy > 0.40 and sx > sy, details=f"sx={sx:.3f}, sy={sy:.3f}")
    ctx.expect_overlap(swivel, swivel, axes="z", elem_a="seat_piping", elem_b="cushion", min_overlap=0.004, name="raised piping sits on blue cushion edge")
    ctx.expect_overlap(swivel, swivel, axes="y", elem_a="rear_hinge_lower_crossmember", elem_b="tilt_mechanism_housing", min_overlap=0.006, name="rear hinge lower crossmember ties into seat chassis")
    for side_name in ("left", "right"):
        ctx.expect_overlap(
            swivel,
            swivel,
            axes="z",
            elem_a=f"rear_hinge_{side_name}_rail_foot",
            elem_b=f"back_hinge_{side_name}_support_rail",
            min_overlap=0.010,
            name=f"{side_name} rear back support rod foot is seated on lower chassis",
        )

    frame_aabb = ctx.part_element_world_aabb(back_frame, elem="backrest_frame")
    bottom_tension_aabb = ctx.part_element_world_aabb(back_panel, elem="mesh_bottom_tension_edge")
    top_tension_aabb = ctx.part_element_world_aabb(back_panel, elem="mesh_top_tension_edge")
    left_tension_aabb = ctx.part_element_world_aabb(back_panel, elem="mesh_left_tension_edge")
    right_tension_aabb = ctx.part_element_world_aabb(back_panel, elem="mesh_right_tension_edge")
    assert frame_aabb is not None and bottom_tension_aabb is not None and top_tension_aabb is not None
    assert left_tension_aabb is not None and right_tension_aabb is not None
    ctx.check("tall mesh back rises above seat", frame_aabb[1][2] - seat_top > 0.48, details=f"back_top={frame_aabb[1][2]:.3f}, seat_top={seat_top:.3f}")
    ctx.check(
        "inner tension net sits inside teal frame",
        left_tension_aabb[0][0] > frame_aabb[0][0]
        and right_tension_aabb[1][0] < frame_aabb[1][0]
        and bottom_tension_aabb[0][2] > frame_aabb[0][2]
        and top_tension_aabb[1][2] < frame_aabb[1][2],
        details=f"frame={frame_aabb}, left={left_tension_aabb}, right={right_tension_aabb}, bottom={bottom_tension_aabb}, top={top_tension_aabb}",
    )
    ctx.check(
        "backrest curves toward the sitter at lumbar height",
        bottom_tension_aabb[1][1] > top_tension_aabb[1][1] + 0.020,
        details=f"bottom_y_max={bottom_tension_aabb[1][1]:.3f}, top_y_max={top_tension_aabb[1][1]:.3f}",
    )
    ctx.check(
        "visible back mesh thread grid",
        all(ctx.part_element_world_aabb(back_frame, elem=f"mesh_h_{i}") is not None for i in range(MESH_H_COUNT))
        and all(ctx.part_element_world_aabb(back_frame, elem=f"mesh_v_{i}") is not None for i in range(MESH_V_COUNT)),
        details=f"expected {MESH_H_COUNT} horizontal and {MESH_V_COUNT} vertical mesh thread parts",
    )
    left_edge_aabb = ctx.part_element_world_aabb(back_frame, elem="mesh_left_tension_edge")
    right_edge_aabb = ctx.part_element_world_aabb(back_frame, elem="mesh_right_tension_edge")
    bottom_edge_aabb = ctx.part_element_world_aabb(back_frame, elem="mesh_bottom_tension_edge")
    top_edge_aabb = ctx.part_element_world_aabb(back_frame, elem="mesh_top_tension_edge")
    h_mid_aabb = ctx.part_element_world_aabb(back_frame, elem=f"mesh_h_{MESH_H_COUNT // 2}")
    v_mid_aabb = ctx.part_element_world_aabb(back_frame, elem=f"mesh_v_{MESH_V_COUNT // 2}")
    assert left_edge_aabb and right_edge_aabb and bottom_edge_aabb and top_edge_aabb and h_mid_aabb and v_mid_aabb
    mid_span = h_mid_aabb[1][0] - h_mid_aabb[0][0]
    top_span = top_edge_aabb[1][0] - top_edge_aabb[0][0]
    bottom_span = bottom_edge_aabb[1][0] - bottom_edge_aabb[0][0]
    ctx.check(
        "mesh net follows rounded frame instead of a square rectangle",
        mid_span > top_span + 0.070 and mid_span > bottom_span + 0.070,
        details=f"mid_span={mid_span:.3f}, top_span={top_span:.3f}, bottom_span={bottom_span:.3f}",
    )
    ctx.check(
        "mesh threads terminate into tension edges",
        h_mid_aabb[0][0] <= left_edge_aabb[1][0] + 0.004
        and h_mid_aabb[1][0] >= right_edge_aabb[0][0] - 0.004
        and v_mid_aabb[0][2] <= bottom_edge_aabb[1][2] + 0.006
        and v_mid_aabb[1][2] >= top_edge_aabb[0][2] - 0.006,
        details=f"h_mid={h_mid_aabb}, v_mid={v_mid_aabb}, left={left_edge_aabb}, right={right_edge_aabb}",
    )
    ctx.check(
        "backrest uses a visible horizontal revolute hinge",
        back_hinge.articulation_type == ArticulationType.REVOLUTE and tuple(round(c, 3) for c in back_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"type={back_hinge.articulation_type}, axis={back_hinge.axis}",
    )
    ctx.expect_overlap(swivel, back_frame, axes="x", elem_a="back_hinge_axle", elem_b="back_hinge_barrel_left", min_overlap=0.060, name="left hinge barrel is mounted on cross axle")
    ctx.expect_overlap(swivel, back_frame, axes="x", elem_a="back_hinge_axle", elem_b="back_hinge_barrel_right", min_overlap=0.060, name="right hinge barrel is mounted on cross axle")
    with ctx.pose({back_hinge: math.radians(12.0)}):
        tilted_frame_aabb = ctx.part_element_world_aabb(back_frame, elem="backrest_frame")
    assert tilted_frame_aabb is not None
    ctx.check(
        "backrest rotates about the new hinge axis",
        abs(tilted_frame_aabb[0][1] - frame_aabb[0][1]) > 0.020 or abs(tilted_frame_aabb[1][2] - frame_aabb[1][2]) > 0.020,
        details=f"rest_frame={frame_aabb}, tilted_frame={tilted_frame_aabb}",
    )

    for side_name in ("left", "right"):
        slider = object_model.get_part(f"{side_name}_armrest_slider")
        joint = object_model.get_articulation(f"{side_name}_armrest_height")
        sleeve_aabb = ctx.part_element_world_aabb(swivel, elem=f"{side_name}_armrest_outer_hollow_sleeve")
        post_aabb = ctx.part_element_world_aabb(slider, elem="sliding_upper_post")
        rim_aabb = ctx.part_element_world_aabb(swivel, elem=f"{side_name}_armrest_top_open_rim")
        ctx.allow_overlap(
            swivel,
            slider,
            elem_a=f"{side_name}_armrest_outer_hollow_sleeve",
            elem_b="sliding_low_friction_bushing",
            reason="Short guide bushing lightly rides inside the hollow height-adjustment sleeve.",
        )
        ctx.expect_overlap(
            swivel,
            slider,
            axes="z",
            elem_a=f"{side_name}_armrest_outer_hollow_sleeve",
            elem_b="sliding_low_friction_bushing",
            min_overlap=0.020,
            name=f"{side_name} guide bushing stays inside hollow sleeve",
        )
        assert sleeve_aabb is not None and post_aabb is not None and rim_aabb is not None
        sleeve_cx = (sleeve_aabb[0][0] + sleeve_aabb[1][0]) * 0.5
        sleeve_cy = (sleeve_aabb[0][1] + sleeve_aabb[1][1]) * 0.5
        post_cx = (post_aabb[0][0] + post_aabb[1][0]) * 0.5
        post_cy = (post_aabb[0][1] + post_aabb[1][1]) * 0.5
        ctx.check(
            f"{side_name} armrest uses visible hollow sleeve around centered slider",
            abs(sleeve_cx - post_cx) < 0.006
            and abs(sleeve_cy - post_cy) < 0.006
            and (rim_aabb[1][0] - rim_aabb[0][0]) > (post_aabb[1][0] - post_aabb[0][0]) + 0.020,
            details=f"sleeve_center=({sleeve_cx:.3f},{sleeve_cy:.3f}), post_center=({post_cx:.3f},{post_cy:.3f})",
        )
        ctx.check(
            f"{side_name} slider is still inside sleeve at rest",
            post_aabb[0][2] < sleeve_aabb[1][2] - 0.025 and post_aabb[1][2] > sleeve_aabb[1][2] + 0.060,
            details=f"post_z={post_aabb[0][2]:.3f}-{post_aabb[1][2]:.3f}, sleeve_z={sleeve_aabb[0][2]:.3f}-{sleeve_aabb[1][2]:.3f}",
        )
        ctx.check(
            f"{side_name} T armrest has vertical prismatic height joint",
            joint.articulation_type == ArticulationType.PRISMATIC and tuple(round(c, 3) for c in joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        pad_aabb = ctx.part_element_world_aabb(slider, elem="rounded_t_pad")
        socket_aabb = ctx.part_element_world_aabb(slider, elem="under_pad_hollow_socket")
        assert pad_aabb is not None and socket_aabb is not None
        pad_x = pad_aabb[1][0] - pad_aabb[0][0]
        pad_y = pad_aabb[1][1] - pad_aabb[0][1]
        ctx.check(
            f"{side_name} armrest has wide T-shaped pad",
            pad_y > 0.22 and pad_x > 0.060 and pad_aabb[0][2] > seat_top + 0.07,
            details=f"pad_x={pad_x:.3f}, pad_y={pad_y:.3f}, pad_bottom={pad_aabb[0][2]:.3f}",
        )
        socket_cx = (socket_aabb[0][0] + socket_aabb[1][0]) * 0.5
        socket_cy = (socket_aabb[0][1] + socket_aabb[1][1]) * 0.5
        ctx.check(
            f"{side_name} arm pad has centered hollow socket for vertical tube",
            abs(socket_cx - post_cx) < 0.004
            and abs(socket_cy - post_cy) < 0.004
            and socket_aabb[0][2] < post_aabb[1][2]
            and socket_aabb[1][2] > post_aabb[1][2] + 0.004
            and socket_aabb[1][2] > pad_aabb[0][2] - 0.004,
            details=f"socket_center=({socket_cx:.3f},{socket_cy:.3f}), post_center=({post_cx:.3f},{post_cy:.3f}), post_top={post_aabb[1][2]:.3f}, socket_z={socket_aabb[0][2]:.3f}-{socket_aabb[1][2]:.3f}",
        )
        ctx.expect_overlap(slider, slider, axes="z", elem_a="under_pad_hollow_socket", elem_b="rounded_t_pad", min_overlap=0.004, name=f"{side_name} hollow socket is attached to arm pad underside")
        before = ctx.part_world_position(slider)
        with ctx.pose({joint: ARM_TRAVEL}):
            after = ctx.part_world_position(slider)
            raised_post_aabb = ctx.part_element_world_aabb(slider, elem="sliding_upper_post")
        assert before is not None and after is not None
        ctx.check(f"{side_name} armrest moves upward under prismatic pose", after[2] - before[2] > ARM_TRAVEL * 0.80, details=f"before={before}, after={after}")
        assert raised_post_aabb is not None
        ctx.check(
            f"{side_name} raised armrest remains captured by hollow sleeve",
            raised_post_aabb[0][2] < sleeve_aabb[1][2] - 0.020,
            details=f"raised_post_bottom={raised_post_aabb[0][2]:.3f}, sleeve_top={sleeve_aabb[1][2]:.3f}",
        )

    col_aabb = ctx.part_element_world_aabb(swivel, elem="gas_lift_column")
    assert col_aabb is not None
    ctx.check("column descends to base hub", col_aabb[0][2] < HUB_Z + 0.02, details=f"column_bottom={col_aabb[0][2]:.3f}")
    ctx.check("column reaches up to seat", col_aabb[1][2] >= SEAT_BOTTOM_Z - 0.01, details=f"column_top={col_aabb[1][2]:.3f}, seat_bottom={SEAT_BOTTOM_Z:.3f}")

    base_aabb = ctx.part_world_aabb(base)
    assert base_aabb is not None
    span_x = base_aabb[1][0] - base_aabb[0][0]
    ctx.check("reused rolling base footprint is wide", span_x > 0.45, details=f"span_x={span_x:.3f}")
    ctx.check(
        "exactly four twin caster pairs are retained",
        len([part for part in object_model.parts if part.name.startswith("caster_wheel_")]) == 8
        and all(any(part.name == f"caster_wheel_{i}_{j}" for part in object_model.parts) for i in range(4) for j in range(2)),
        details="expected caster_wheel_i_j for i=0..3 and j=0..1 only",
    )

    centers = []
    for i in range(N_SPOKES):
        w_left = ctx.part_world_position(object_model.get_part(f"caster_wheel_{i}_0"))
        w_right = ctx.part_world_position(object_model.get_part(f"caster_wheel_{i}_1"))
        assert w_left is not None and w_right is not None
        centers.append(((w_left[0] + w_right[0]) * 0.5, (w_left[1] + w_right[1]) * 0.5))
    angle_steps = []
    for i in range(N_SPOKES):
        a0 = math.atan2(centers[i][1], centers[i][0])
        a1 = math.atan2(centers[(i + 1) % N_SPOKES][1], centers[(i + 1) % N_SPOKES][0])
        angle_steps.append((a1 - a0) % (2.0 * math.pi))
    ctx.check("four caster pairs remain evenly spaced", all(abs(step - math.pi / 2.0) < 0.02 for step in angle_steps), details=f"angle_steps={[round(step, 3) for step in angle_steps]}")

    for i in range(N_SPOKES):
        for j in range(2):
            wa = ctx.part_world_aabb(object_model.get_part(f"caster_wheel_{i}_{j}"))
            assert wa is not None
            ctx.check(f"caster wheel {i}_{j} near floor", wa[0][2] < 0.008, details=f"wheel_bottom={wa[0][2]:.3f}")

    ctx.check(
        "base_swivel is continuous about Z",
        base_swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(round(c, 3) for c in base_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={base_swivel.articulation_type}, axis={base_swivel.axis}",
    )
    caster_before = ctx.part_world_position(object_model.get_part("caster_wheel_0_0"))
    with ctx.pose({base_swivel: math.pi / 2.0}):
        caster_after = ctx.part_world_position(object_model.get_part("caster_wheel_0_0"))
        ctx.expect_contact(bearing, base, elem_a="bearing_washer", elem_b="hub", name="base remains seated on bearing while swiveled")
    assert caster_before is not None and caster_after is not None
    ctx.check(
        "base swivel rotates caster-base assembly",
        abs(caster_after[0] + caster_before[1]) < 0.015 and abs(caster_after[1] - caster_before[0]) < 0.015,
        details=f"before={caster_before}, after={caster_after}",
    )

    ctx.check(
        "seat_swivel is continuous about Z",
        seat_swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(round(c, 3) for c in seat_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={seat_swivel.articulation_type}, axis={seat_swivel.axis}",
    )
    back_before = ctx.part_world_position(back_frame)
    with ctx.pose({seat_swivel: math.pi / 2.0}):
        back_after = ctx.part_world_position(back_frame)
        ctx.expect_within(swivel, base, axes="xy", inner_elem="gas_lift_column", margin=0.35, name="column stays over base under swivel")
    assert back_before is not None and back_after is not None
    ctx.check("seat swivel carries the upper chair and backrest", back_after is not None, details=f"before={back_before}, after={back_after}")

    roll = object_model.get_articulation("caster_roll_0_0")
    ctx.check("caster roll is continuous", roll.articulation_type == ArticulationType.CONTINUOUS, details=f"type={roll.articulation_type}")
    w0 = object_model.get_part("caster_wheel_0_0")
    pos_before = ctx.part_world_position(w0)
    with ctx.pose({roll: 1.0}):
        pos_after = ctx.part_world_position(w0)
    ctx.check("caster roll pose applies", pos_before is not None and pos_after is not None, details=f"before={pos_before}, after={pos_after}")

    return ctx.report()


object_model = build_object_model()
