from __future__ import annotations

# >>> USER_CODE_START
import math

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
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

# Coordinates: +X is parallel to the wheel's spin axis (between the two
# towers), +Y is "forward" toward the loading platform, +Z is up. The base
# plate rests on z=0.

NUM_GONDOLAS = 12
ANGLE_OFFSET = -math.pi / 2.0     # gondola 00 starts at the bottom of the wheel

# Wheel sizing.
AXLE_Z = 0.92
RIM_RADIUS = 0.50
RIM_TUBE = 0.014
WHEEL_HALF_WIDTH = 0.078           # half the distance between the two rim planes
PIVOT_RADIUS = 0.68                # outside the rim so cabins clear the rim plane

AXLE_SHAFT_RADIUS = 0.018
# Long enough to seat well inside both bearings.
AXLE_SHAFT_LENGTH = 0.466
HUB_BARREL_RADIUS = 0.055
HUB_BARREL_LENGTH = WHEEL_HALF_WIDTH * 2.0 * 0.9
HUB_FLANGE_RADIUS = 0.090
HUB_FLANGE_THICKNESS = 0.012

# Support frame.
SUPPORT_X = 0.215                  # tower head X offset from axle
BASE_X_HALF = 0.40
BASE_Y_HALF = 0.30
BASE_THICKNESS = 0.040
TOWER_TOP_Z = AXLE_Z - 0.14        # headstock top sits just under the bearing
TOWER_FOOT_Z = BASE_THICKNESS
TOWER_FOOT_OUT = 0.22              # outboard Y span of tower legs at base
TOWER_FOOT_IN = -0.22              # inboard Y span (mirror)

HEADSTOCK_W = 0.082
HEADSTOCK_D = 0.110
HEADSTOCK_H = 0.110
BEARING_W = 0.040
BEARING_D = 0.066
BEARING_H = 0.060

LOADING_PLATFORM_DEPTH = 0.24
LOADING_PLATFORM_Y_CENTER = BASE_Y_HALF + LOADING_PLATFORM_DEPTH * 0.5
LOADING_PLATFORM_HEIGHT = 0.090
LOADING_PLATFORM_WIDTH_HALF = 0.28
LOADING_STEP_HEIGHT = 0.045
LOADING_STEP_WIDTH_HALF = 0.18
LOADING_STEP_DEPTH = 0.10

MOTOR_BOX_W = 0.090
MOTOR_BOX_D = 0.110
MOTOR_BOX_H = 0.130
MOTOR_BOX_X = -(SUPPORT_X + MOTOR_BOX_W * 0.5 + 0.012)
MOTOR_BOX_Z = TOWER_TOP_Z - MOTOR_BOX_H * 0.5

# Gondolas. Each gondola's part frame sits at its pivot; geometry hangs in -Z.
GONDOLA_PIVOT_BARREL_LENGTH = 0.118  # spans between the two cheeks
GONDOLA_PIVOT_BARREL_RADIUS = 0.0070
GONDOLA_PIVOT_CHEEK_W = 0.010
GONDOLA_PIVOT_CHEEK_D = 0.026
GONDOLA_PIVOT_CHEEK_H = 0.034

GONDOLA_SLEEVE_LENGTH = 0.070
GONDOLA_SLEEVE_RADIUS = 0.0098
GONDOLA_HANGER_X = 0.024
GONDOLA_HANGER_LENGTH = 0.056
GONDOLA_HANGER_RADIUS = 0.0040
GONDOLA_ROOF_BEAM_W = 0.094
GONDOLA_ROOF_BEAM_D = 0.013
GONDOLA_ROOF_BEAM_H = 0.012
GONDOLA_ROOF_SHELL_W = 0.118
GONDOLA_ROOF_SHELL_D = 0.094
GONDOLA_ROOF_SHELL_H = 0.014
GONDOLA_BODY_W = 0.110
GONDOLA_BODY_D = 0.086
GONDOLA_BODY_H = 0.060
GONDOLA_FLOOR_THICK = 0.008
GONDOLA_WALL_THICK = 0.006

GONDOLA_POST_RADIUS = 0.0040
GONDOLA_RAIL_RADIUS = 0.0040
GONDOLA_SEAT_W = 0.092
GONDOLA_SEAT_D = 0.046
GONDOLA_SEAT_H = 0.010
GONDOLA_SAFETY_BAR_RADIUS = 0.0042

# Z layering in gondola local frame (origin at pivot).
GONDOLA_Z_HANGER_TOP = -GONDOLA_SLEEVE_RADIUS
GONDOLA_Z_HANGER_BOTTOM = GONDOLA_Z_HANGER_TOP - GONDOLA_HANGER_LENGTH
# The hanger ends embed in the center of the roof beam (small intentional
# overlap that keeps the two parts visually fused).
GONDOLA_Z_ROOF_BEAM_CENTER = GONDOLA_Z_HANGER_BOTTOM
GONDOLA_Z_ROOF_BEAM_BOTTOM = GONDOLA_Z_ROOF_BEAM_CENTER - GONDOLA_ROOF_BEAM_H * 0.5
GONDOLA_Z_ROOF_SHELL_TOP = GONDOLA_Z_ROOF_BEAM_BOTTOM + 0.001
GONDOLA_Z_ROOF_SHELL_CENTER = GONDOLA_Z_ROOF_SHELL_TOP - GONDOLA_ROOF_SHELL_H * 0.5
GONDOLA_Z_ROOF_SHELL_BOTTOM = GONDOLA_Z_ROOF_SHELL_TOP - GONDOLA_ROOF_SHELL_H
GONDOLA_Z_BODY_TOP = GONDOLA_Z_ROOF_SHELL_BOTTOM
GONDOLA_Z_BODY_CENTER = GONDOLA_Z_BODY_TOP - GONDOLA_BODY_H * 0.5
GONDOLA_Z_BODY_BOTTOM = GONDOLA_Z_BODY_TOP - GONDOLA_BODY_H
GONDOLA_Z_FLOOR_CENTER = GONDOLA_Z_BODY_BOTTOM - GONDOLA_FLOOR_THICK * 0.5
GONDOLA_Z_SEAT_CENTER = GONDOLA_Z_BODY_CENTER - 0.006
GONDOLA_Z_SAFETY_BAR = GONDOLA_Z_BODY_CENTER + 0.012
GONDOLA_Z_TOP_RAIL = GONDOLA_Z_BODY_TOP - 0.005

GONDOLA_SWING_LIMIT = math.pi


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _midpoint(a, b):
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius, material, name):
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _wheel_point(radius, angle, x=0.0):
    return (x, radius * math.cos(angle), radius * math.sin(angle))


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return (
        (low[0] + high[0]) * 0.5,
        (low[1] + high[1]) * 0.5,
        (low[2] + high[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ferris_wheel", assets=ASSETS)

    base_paint = model.material("base_paint", rgba=(0.34, 0.36, 0.40, 1.0))
    tower_paint = model.material("tower_paint", rgba=(0.86, 0.88, 0.90, 1.0))
    wheel_paint = model.material("wheel_paint", rgba=(0.93, 0.94, 0.95, 1.0))
    hub_paint = model.material("hub_paint", rgba=(0.30, 0.32, 0.36, 1.0))
    axle_gray = model.material("axle_gray", rgba=(0.30, 0.32, 0.36, 1.0))
    motor_paint = model.material("motor_paint", rgba=(0.62, 0.18, 0.18, 1.0))
    platform_paint = model.material("platform_paint", rgba=(0.42, 0.30, 0.22, 1.0))
    gondola_red = model.material("gondola_red", rgba=(0.78, 0.15, 0.16, 1.0))
    gondola_cream = model.material("gondola_cream", rgba=(0.94, 0.92, 0.86, 1.0))
    gondola_seat = model.material("gondola_seat", rgba=(0.20, 0.16, 0.13, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.55, 0.57, 0.61, 1.0))

    rim_mesh = _save_mesh(
        "ferris_wheel_rim.obj",
        TorusGeometry(
            radius=RIM_RADIUS,
            tube=RIM_TUBE,
            radial_segments=22,
            tubular_segments=128,
        ).rotate_y(math.pi / 2.0),
    )
    # ==================== Base / Support Frame ====================
    support_frame = model.part("support_frame")

    support_frame.visual(
        Box((BASE_X_HALF * 2.0, BASE_Y_HALF * 2.0, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material=base_paint,
        name="base_plate",
    )
    for sign, name in ((-1.0, "rear_base_rib"), (1.0, "front_base_rib")):
        support_frame.visual(
            Box((BASE_X_HALF * 1.85, 0.040, 0.030)),
            origin=Origin(
                xyz=(0.0, sign * (BASE_Y_HALF - 0.040), BASE_THICKNESS + 0.015)
            ),
            material=base_paint,
            name=name,
        )

    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        tower_top = (side_sign * SUPPORT_X, 0.0, TOWER_TOP_Z)
        front_foot = (side_sign * SUPPORT_X, TOWER_FOOT_OUT, TOWER_FOOT_Z)
        rear_foot = (side_sign * SUPPORT_X, TOWER_FOOT_IN, TOWER_FOOT_Z)
        _add_member(
            support_frame,
            front_foot,
            tower_top,
            radius=0.0145,
            material=tower_paint,
            name=f"{side_name}_front_leg",
        )
        _add_member(
            support_frame,
            rear_foot,
            tower_top,
            radius=0.0145,
            material=tower_paint,
            name=f"{side_name}_rear_leg",
        )
        mid_z_low = TOWER_FOOT_Z + (TOWER_TOP_Z - TOWER_FOOT_Z) * 0.30
        mid_z_high = TOWER_FOOT_Z + (TOWER_TOP_Z - TOWER_FOOT_Z) * 0.70
        for tag, ratio in (("lower_tie", 0.30), ("upper_tie", 0.70)):
            z = TOWER_FOOT_Z + (TOWER_TOP_Z - TOWER_FOOT_Z) * ratio
            y_at_z = TOWER_FOOT_OUT * (1.0 - ratio)
            tie_a = (side_sign * SUPPORT_X, y_at_z, z)
            tie_b = (side_sign * SUPPORT_X, -y_at_z, z)
            _add_member(
                support_frame,
                tie_a,
                tie_b,
                radius=0.0085,
                material=tower_paint,
                name=f"{side_name}_{tag}",
            )
        x_low_front = (side_sign * SUPPORT_X, TOWER_FOOT_OUT * 0.7, mid_z_low)
        x_low_rear = (side_sign * SUPPORT_X, -TOWER_FOOT_OUT * 0.7, mid_z_low)
        x_high_front = (side_sign * SUPPORT_X, TOWER_FOOT_OUT * 0.3, mid_z_high)
        x_high_rear = (side_sign * SUPPORT_X, -TOWER_FOOT_OUT * 0.3, mid_z_high)
        _add_member(
            support_frame,
            x_low_front,
            x_high_rear,
            radius=0.0070,
            material=tower_paint,
            name=f"{side_name}_x_brace_a",
        )
        _add_member(
            support_frame,
            x_low_rear,
            x_high_front,
            radius=0.0070,
            material=tower_paint,
            name=f"{side_name}_x_brace_b",
        )
        support_frame.visual(
            Box((HEADSTOCK_W, HEADSTOCK_D, HEADSTOCK_H)),
            origin=Origin(
                xyz=(side_sign * SUPPORT_X, 0.0, TOWER_TOP_Z + HEADSTOCK_H * 0.5)
            ),
            material=tower_paint,
            name=f"{side_name}_headstock",
        )
        support_frame.visual(
            Box((BEARING_W, BEARING_D, BEARING_H)),
            origin=Origin(
                xyz=(side_sign * (SUPPORT_X - 0.005), 0.0, AXLE_Z)
            ),
            material=hub_paint,
            name=f"{side_name}_bearing",
        )

    support_frame.visual(
        Box((MOTOR_BOX_W, MOTOR_BOX_D, MOTOR_BOX_H)),
        origin=Origin(xyz=(MOTOR_BOX_X, 0.0, MOTOR_BOX_Z)),
        material=motor_paint,
        name="motor_housing",
    )
    _add_member(
        support_frame,
        (MOTOR_BOX_X + MOTOR_BOX_W * 0.5, 0.0, MOTOR_BOX_Z - MOTOR_BOX_H * 0.30),
        (-SUPPORT_X, 0.0, TOWER_TOP_Z),
        radius=0.0075,
        material=tower_paint,
        name="motor_strut",
    )

    support_frame.visual(
        Box(
            (
                LOADING_PLATFORM_WIDTH_HALF * 2.0,
                LOADING_PLATFORM_DEPTH,
                LOADING_PLATFORM_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(0.0, LOADING_PLATFORM_Y_CENTER, LOADING_PLATFORM_HEIGHT * 0.5)
        ),
        material=platform_paint,
        name="loading_platform",
    )
    step_y = LOADING_PLATFORM_Y_CENTER + LOADING_PLATFORM_DEPTH * 0.5 + LOADING_STEP_DEPTH * 0.5
    support_frame.visual(
        Box((LOADING_STEP_WIDTH_HALF * 2.0, LOADING_STEP_DEPTH, LOADING_STEP_HEIGHT)),
        origin=Origin(xyz=(0.0, step_y, LOADING_STEP_HEIGHT * 0.5)),
        material=platform_paint,
        name="loading_step",
    )

    rail_y = LOADING_PLATFORM_Y_CENTER + LOADING_PLATFORM_DEPTH * 0.5 - 0.018
    rail_top_z = LOADING_PLATFORM_HEIGHT + 0.18
    for tag, x_sign in (("loading_rail_left_post", -1.0), ("loading_rail_right_post", 1.0)):
        post_x = x_sign * (LOADING_PLATFORM_WIDTH_HALF - 0.020)
        _add_member(
            support_frame,
            (post_x, rail_y, LOADING_PLATFORM_HEIGHT),
            (post_x, rail_y, rail_top_z),
            radius=0.005,
            material=rail_metal,
            name=tag,
        )
    _add_member(
        support_frame,
        (-(LOADING_PLATFORM_WIDTH_HALF - 0.020), rail_y, rail_top_z),
        ((LOADING_PLATFORM_WIDTH_HALF - 0.020), rail_y, rail_top_z),
        radius=0.005,
        material=rail_metal,
        name="loading_rail_top",
    )

    support_frame.inertial = Inertial.from_geometry(
        Box(
            (
                BASE_X_HALF * 2.0,
                BASE_Y_HALF * 2.0 + LOADING_PLATFORM_DEPTH,
                AXLE_Z,
            )
        ),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z * 0.35)),
    )

    # ==================== Rotating Wheel ====================
    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=AXLE_SHAFT_RADIUS, length=AXLE_SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_gray,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=HUB_BARREL_RADIUS, length=HUB_BARREL_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_paint,
        name="hub_barrel",
    )
    for sign, tag in ((-1.0, "rear_hub_flange"), (1.0, "front_hub_flange")):
        wheel.visual(
            Cylinder(radius=HUB_FLANGE_RADIUS, length=HUB_FLANGE_THICKNESS),
            origin=Origin(
                xyz=(sign * (WHEEL_HALF_WIDTH - HUB_FLANGE_THICKNESS * 0.5), 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hub_paint,
            name=tag,
        )

    for sign, tag in ((1.0, "front_rim"), (-1.0, "rear_rim")):
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(sign * WHEEL_HALF_WIDTH, 0.0, 0.0)),
            material=wheel_paint,
            name=tag,
        )

    spoke_inner_r = HUB_FLANGE_RADIUS - 0.004
    spoke_outer_r = RIM_RADIUS - RIM_TUBE
    for index in range(NUM_GONDOLAS):
        angle = ANGLE_OFFSET + (2.0 * math.pi * index) / NUM_GONDOLAS
        for sign, tag in ((1.0, "front_spoke"), (-1.0, "rear_spoke")):
            plane_x = sign * WHEEL_HALF_WIDTH
            _add_member(
                wheel,
                _wheel_point(spoke_inner_r, angle, plane_x),
                _wheel_point(spoke_outer_r, angle, plane_x),
                radius=0.0048,
                material=wheel_paint,
                name=f"{tag}_{index:02d}",
            )

    # Axial inter-rim spacers, located angularly midway between gondolas so
    # they sit clear of the cabin swept volume.
    for index in range(NUM_GONDOLAS):
        spacer_angle = ANGLE_OFFSET + (2.0 * math.pi * (index + 0.5)) / NUM_GONDOLAS
        sp = _wheel_point(RIM_RADIUS - RIM_TUBE * 0.8, spacer_angle, 0.0)
        _add_member(
            wheel,
            (-WHEEL_HALF_WIDTH, sp[1], sp[2]),
            (WHEEL_HALF_WIDTH, sp[1], sp[2]),
            radius=0.0042,
            material=wheel_paint,
            name=f"inter_rim_spacer_{index:02d}",
        )

    for index in range(NUM_GONDOLAS):
        angle = ANGLE_OFFSET + (2.0 * math.pi * index) / NUM_GONDOLAS
        pivot_point = _wheel_point(PIVOT_RADIUS, angle, 0.0)
        rim_join_front = _wheel_point(RIM_RADIUS - RIM_TUBE * 0.45, angle, WHEEL_HALF_WIDTH)
        rim_join_rear = _wheel_point(RIM_RADIUS - RIM_TUBE * 0.45, angle, -WHEEL_HALF_WIDTH)
        front_cheek = (WHEEL_HALF_WIDTH - 0.014, pivot_point[1], pivot_point[2])
        rear_cheek = (-(WHEEL_HALF_WIDTH - 0.014), pivot_point[1], pivot_point[2])
        wheel.visual(
            Cylinder(radius=GONDOLA_PIVOT_BARREL_RADIUS, length=GONDOLA_PIVOT_BARREL_LENGTH),
            origin=Origin(xyz=pivot_point, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=axle_gray,
            name=f"pivot_barrel_{index:02d}",
        )
        wheel.visual(
            Box((GONDOLA_PIVOT_CHEEK_W, GONDOLA_PIVOT_CHEEK_D, GONDOLA_PIVOT_CHEEK_H)),
            origin=Origin(xyz=front_cheek),
            material=hub_paint,
            name=f"front_pivot_cheek_{index:02d}",
        )
        wheel.visual(
            Box((GONDOLA_PIVOT_CHEEK_W, GONDOLA_PIVOT_CHEEK_D, GONDOLA_PIVOT_CHEEK_H)),
            origin=Origin(xyz=rear_cheek),
            material=hub_paint,
            name=f"rear_pivot_cheek_{index:02d}",
        )
        _add_member(
            wheel,
            rim_join_front,
            front_cheek,
            radius=0.0042,
            material=hub_paint,
            name=f"front_pivot_strut_{index:02d}",
        )
        _add_member(
            wheel,
            rim_join_rear,
            rear_cheek,
            radius=0.0042,
            material=hub_paint,
            name=f"rear_pivot_strut_{index:02d}",
        )

    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=RIM_RADIUS, length=WHEEL_HALF_WIDTH * 2.0),
        mass=28.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2),
    )

    # ==================== Gondolas (12) ====================
    for index in range(NUM_GONDOLAS):
        gondola = model.part(f"gondola_{index:02d}")

        gondola.visual(
            Cylinder(radius=GONDOLA_SLEEVE_RADIUS, length=GONDOLA_SLEEVE_LENGTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=axle_gray,
            name="pivot_sleeve",
        )
        _add_member(
            gondola,
            (-GONDOLA_HANGER_X, 0.0, GONDOLA_Z_HANGER_TOP),
            (-GONDOLA_HANGER_X, 0.0, GONDOLA_Z_HANGER_BOTTOM),
            radius=GONDOLA_HANGER_RADIUS,
            material=axle_gray,
            name="left_hanger",
        )
        _add_member(
            gondola,
            (GONDOLA_HANGER_X, 0.0, GONDOLA_Z_HANGER_TOP),
            (GONDOLA_HANGER_X, 0.0, GONDOLA_Z_HANGER_BOTTOM),
            radius=GONDOLA_HANGER_RADIUS,
            material=axle_gray,
            name="right_hanger",
        )
        gondola.visual(
            Box((GONDOLA_ROOF_BEAM_W, GONDOLA_ROOF_BEAM_D, GONDOLA_ROOF_BEAM_H)),
            origin=Origin(xyz=(0.0, 0.0, GONDOLA_Z_ROOF_BEAM_CENTER)),
            material=axle_gray,
            name="roof_beam",
        )
        # Painted sheet-metal roof shell, overhanging the cabin walls.
        gondola.visual(
            Box((GONDOLA_ROOF_SHELL_W, GONDOLA_ROOF_SHELL_D, GONDOLA_ROOF_SHELL_H)),
            origin=Origin(xyz=(0.0, 0.0, GONDOLA_Z_ROOF_SHELL_CENTER)),
            material=gondola_red,
            name="roof_shell",
        )
        gondola.visual(
            Box((GONDOLA_BODY_W, GONDOLA_BODY_D, GONDOLA_FLOOR_THICK)),
            origin=Origin(xyz=(0.0, 0.0, GONDOLA_Z_FLOOR_CENTER)),
            material=gondola_red,
            name="floor_pan",
        )
        for sign, tag in ((-1.0, "left_wall"), (1.0, "right_wall")):
            gondola.visual(
                Box((GONDOLA_WALL_THICK, GONDOLA_BODY_D, GONDOLA_BODY_H * 0.45)),
                origin=Origin(
                    xyz=(
                        sign * (GONDOLA_BODY_W * 0.5 - GONDOLA_WALL_THICK * 0.5),
                        0.0,
                        GONDOLA_Z_BODY_CENTER - GONDOLA_BODY_H * 0.275,
                    )
                ),
                material=gondola_cream,
                name=tag,
            )
        gondola.visual(
            Box((GONDOLA_BODY_W, GONDOLA_WALL_THICK, GONDOLA_BODY_H)),
            origin=Origin(
                xyz=(
                    0.0,
                    -GONDOLA_BODY_D * 0.5 + GONDOLA_WALL_THICK * 0.5,
                    GONDOLA_Z_BODY_CENTER,
                )
            ),
            material=gondola_cream,
            name="rear_wall",
        )
        post_top_z = GONDOLA_Z_BODY_TOP - 0.002
        post_bottom_z = GONDOLA_Z_FLOOR_CENTER + GONDOLA_FLOOR_THICK * 0.5
        for x_sign, y_sign, tag in (
            (-1.0, 1.0, "front_left_post"),
            (1.0, 1.0, "front_right_post"),
            (-1.0, -1.0, "rear_left_post"),
            (1.0, -1.0, "rear_right_post"),
        ):
            px = x_sign * (GONDOLA_BODY_W * 0.5 - GONDOLA_POST_RADIUS - 0.001)
            py = y_sign * (GONDOLA_BODY_D * 0.5 - GONDOLA_POST_RADIUS - 0.001)
            _add_member(
                gondola,
                (px, py, post_bottom_z),
                (px, py, post_top_z),
                radius=GONDOLA_POST_RADIUS,
                material=gondola_red,
                name=tag,
            )
        front_y = GONDOLA_BODY_D * 0.5 - GONDOLA_POST_RADIUS - 0.001
        rail_x = GONDOLA_BODY_W * 0.5 - GONDOLA_POST_RADIUS - 0.001
        _add_member(
            gondola,
            (-rail_x, front_y, GONDOLA_Z_TOP_RAIL),
            (rail_x, front_y, GONDOLA_Z_TOP_RAIL),
            radius=GONDOLA_RAIL_RADIUS,
            material=gondola_red,
            name="front_top_rail",
        )
        for x_sign, tag in ((-1.0, "left_top_rail"), (1.0, "right_top_rail")):
            x = x_sign * rail_x
            _add_member(
                gondola,
                (x, -front_y, GONDOLA_Z_TOP_RAIL),
                (x, front_y, GONDOLA_Z_TOP_RAIL),
                radius=GONDOLA_RAIL_RADIUS,
                material=gondola_red,
                name=tag,
            )
        gondola.visual(
            Box((GONDOLA_SEAT_W, GONDOLA_SEAT_D, GONDOLA_SEAT_H)),
            origin=Origin(
                xyz=(
                    0.0,
                    -GONDOLA_BODY_D * 0.25,
                    GONDOLA_Z_SEAT_CENTER,
                )
            ),
            material=gondola_seat,
            name="seat",
        )
        gondola.visual(
            Box((GONDOLA_SEAT_W, GONDOLA_WALL_THICK, GONDOLA_BODY_H * 0.40)),
            origin=Origin(
                xyz=(
                    0.0,
                    -GONDOLA_BODY_D * 0.5 + GONDOLA_WALL_THICK * 1.5,
                    GONDOLA_Z_SEAT_CENTER + GONDOLA_BODY_H * 0.16,
                )
            ),
            material=gondola_seat,
            name="seat_back",
        )
        _add_member(
            gondola,
            (-rail_x, GONDOLA_BODY_D * 0.5 - 0.012, GONDOLA_Z_SAFETY_BAR),
            (rail_x, GONDOLA_BODY_D * 0.5 - 0.012, GONDOLA_Z_SAFETY_BAR),
            radius=GONDOLA_SAFETY_BAR_RADIUS,
            material=rail_metal,
            name="safety_bar",
        )

        gondola.inertial = Inertial.from_geometry(
            Box(
                (
                    GONDOLA_BODY_W,
                    GONDOLA_BODY_D,
                    GONDOLA_BODY_H + GONDOLA_HANGER_LENGTH,
                )
            ),
            mass=3.6,
            origin=Origin(xyz=(0.0, 0.0, GONDOLA_Z_BODY_CENTER)),
        )

        angle = ANGLE_OFFSET + (2.0 * math.pi * index) / NUM_GONDOLAS
        model.articulation(
            f"wheel_to_gondola_{index:02d}",
            ArticulationType.REVOLUTE,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=_wheel_point(PIVOT_RADIUS, angle)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=3.0,
                lower=-GONDOLA_SWING_LIMIT,
                upper=GONDOLA_SWING_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support_frame = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("support_to_wheel")
    axle_shaft = wheel.get_visual("axle_shaft")
    left_bearing = support_frame.get_visual("left_bearing")
    right_bearing = support_frame.get_visual("right_bearing")

    gondolas = [object_model.get_part(f"gondola_{index:02d}") for index in range(NUM_GONDOLAS)]
    gondola_joints = [
        object_model.get_articulation(f"wheel_to_gondola_{index:02d}")
        for index in range(NUM_GONDOLAS)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        support_frame,
        wheel,
        reason="The axle shaft seats inside the left bearing block.",
        elem_a=left_bearing,
        elem_b=axle_shaft,
    )
    ctx.allow_overlap(
        support_frame,
        wheel,
        reason="The axle shaft seats inside the right bearing block.",
        elem_a=right_bearing,
        elem_b=axle_shaft,
    )
    for index, gondola in enumerate(gondolas):
        ctx.allow_overlap(
            gondola,
            wheel,
            reason="The gondola pivot sleeve wraps the wheel pivot barrel.",
            elem_a=gondola.get_visual("pivot_sleeve"),
            elem_b=wheel.get_visual(f"pivot_barrel_{index:02d}"),
        )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    support_aabb = ctx.part_world_aabb(support_frame)
    if support_aabb is not None:
        smin, smax = support_aabb
        ctx.check(
            "base_grounded_on_floor",
            abs(smin[2]) <= 1e-5,
            f"support_frame should rest on z=0, got min_z={smin[2]:.6f}",
        )
        ctx.check(
            "base_realistic_extent_x",
            (smax[0] - smin[0]) >= 0.45,
            f"support_frame X extent {smax[0] - smin[0]:.3f} too small",
        )
        ctx.check(
            "base_realistic_extent_y",
            (smax[1] - smin[1]) >= 0.55,
            f"support_frame Y extent {smax[1] - smin[1]:.3f} too small",
        )

    ctx.expect_origin_distance(
        wheel,
        support_frame,
        axes="xy",
        max_dist=0.001,
        name="wheel_axle_centered_between_towers",
    )
    ctx.expect_origin_gap(
        wheel,
        support_frame,
        axis="z",
        min_gap=AXLE_Z - 0.005,
        max_gap=AXLE_Z + 0.005,
        name="wheel_axle_height_above_base",
    )
    ctx.expect_contact(
        wheel,
        support_frame,
        elem_a=axle_shaft,
        elem_b=left_bearing,
        name="axle_seats_in_left_bearing",
    )
    ctx.expect_contact(
        wheel,
        support_frame,
        elem_a=axle_shaft,
        elem_b=right_bearing,
        name="axle_seats_in_right_bearing",
    )

    pivot_barrel_00 = wheel.get_visual("pivot_barrel_00")
    rest_pivot_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem=pivot_barrel_00))
    if rest_pivot_center is None:
        ctx.fail("pivot_barrel_00_measurable", "pivot_barrel_00 has no measurable AABB")
    else:
        ctx.check(
            "bottom_pivot_starts_below_axle",
            abs(rest_pivot_center[1]) < 0.015
            and abs(rest_pivot_center[2] - (AXLE_Z - PIVOT_RADIUS)) < 0.02,
            f"pivot_barrel_00 center={rest_pivot_center}",
        )

    for index, gondola in enumerate(gondolas):
        sleeve = gondola.get_visual("pivot_sleeve")
        barrel = wheel.get_visual(f"pivot_barrel_{index:02d}")
        ctx.expect_contact(
            gondola,
            wheel,
            elem_a=sleeve,
            elem_b=barrel,
            name=f"gondola_{index:02d}_pivot_contact_rest",
        )
        pivot_pos = ctx.part_world_position(gondola)
        floor_center = _aabb_center(
            ctx.part_element_world_aabb(gondola, elem=gondola.get_visual("floor_pan"))
        )
        if pivot_pos is None or floor_center is None:
            ctx.fail(
                f"gondola_{index:02d}_measurable_rest",
                f"pivot_pos={pivot_pos}, floor_pan_center={floor_center}",
            )
            continue
        ctx.check(
            f"gondola_{index:02d}_hangs_below_pivot_rest",
            floor_center[2] < pivot_pos[2] - 0.08
            and abs(floor_center[1] - pivot_pos[1]) < 0.02,
            f"pivot={pivot_pos}, floor_pan={floor_center}",
        )

    with ctx.pose({wheel_spin: math.pi / 2.0}):
        turned_pivot = _aabb_center(ctx.part_element_world_aabb(wheel, elem=pivot_barrel_00))
        if turned_pivot is None:
            ctx.fail("wheel_quarter_turn_measurable", "pivot_barrel_00 missing in quarter turn")
        else:
            ctx.check(
                "wheel_quarter_turn_moves_bottom_pivot_to_side",
                turned_pivot[1] > PIVOT_RADIUS - 0.03
                and abs(turned_pivot[2] - AXLE_Z) < 0.025,
                f"turned_pivot={turned_pivot}",
            )

    compensation_pose = {wheel_spin: math.pi / 2.0}
    for gondola_joint in gondola_joints:
        compensation_pose[gondola_joint] = -math.pi / 2.0
    with ctx.pose(compensation_pose):
        for index, gondola in enumerate(gondolas):
            pivot_pos = ctx.part_world_position(gondola)
            floor_center = _aabb_center(
                ctx.part_element_world_aabb(gondola, elem=gondola.get_visual("floor_pan"))
            )
            if pivot_pos is None or floor_center is None:
                ctx.fail(
                    f"gondola_{index:02d}_measurable_quarter",
                    f"pivot_pos={pivot_pos}, floor_center={floor_center}",
                )
                continue
            ctx.check(
                f"gondola_{index:02d}_upright_at_quarter_turn",
                floor_center[2] < pivot_pos[2] - 0.07,
                f"pivot={pivot_pos}, floor_center={floor_center}",
            )
            ctx.expect_contact(
                gondola,
                wheel,
                elem_a=gondola.get_visual("pivot_sleeve"),
                elem_b=wheel.get_visual(f"pivot_barrel_{index:02d}"),
                name=f"gondola_{index:02d}_pivot_contact_quarter",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
