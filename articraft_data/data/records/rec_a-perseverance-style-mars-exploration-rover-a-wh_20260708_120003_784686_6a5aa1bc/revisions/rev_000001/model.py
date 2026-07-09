from __future__ import annotations

# Perseverance-style Mars exploration rover.
#
# Layout convention: +X = forward (camera mast / robotic arm end), +Y = left,
# +Z = up, ground plane at z = 0. Dimensions approximate the real rover
# (~2.2 m body, 0.52 m diameter wheels, mast head ~2.2 m above ground).

from math import pi

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
    TireGeometry,
    TireSidewall,
    TireTread,
    WheelFace,
    WheelGeometry,
    WheelHub,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Parameterized rocker-bogie geometry
# ---------------------------------------------------------------------------
BODY_CENTER = (0.10, 0.0, 0.95)
BODY_SIZE = (2.20, 1.40, 0.60)
BODY_TOP = BODY_CENTER[2] + BODY_SIZE[2] / 2.0  # 1.25
BODY_HALF_W = BODY_SIZE[1] / 2.0  # 0.70

DECK_SIZE = (2.00, 1.30, 0.06)
DECK_CENTER = (0.10, 0.0, BODY_TOP + DECK_SIZE[2] / 2.0)
DECK_TOP = BODY_TOP + DECK_SIZE[2]  # 1.31

WHEEL_RADIUS = 0.26
WHEEL_WIDTH = 0.20
WHEEL_TRACK_Y = 0.95  # lateral distance from centerline to wheel center
WHEEL_ROWS = ("front", "middle", "rear")
WHEEL_ROW_X = (1.05, 0.05, -0.95)  # front / middle / rear axle stations
AXLE_RADIUS = 0.030
AXLE_INNER_Y = 0.72  # where the suspension arm carries the axle
AXLE_TIP_Y = 0.93  # axle tip embedded inside the wheel hub

ROCKER_PIVOT = (0.25, 0.74, 0.95)  # rocker-to-body differential pivot
BOGIE_PIVOT = (-0.45, 0.80, 0.60)  # rocker-to-bogie pivot
SUSP_TUBE_R = 0.034

MAST_BASE = (0.75, 0.15, DECK_TOP)
MAST_HEIGHT = 0.88  # deck top to head-tilt axis
ARM_SHOULDER = (1.32, -0.45, 0.85)
HGA_BASE = (0.10, 0.45, DECK_TOP + 0.10)  # top of the gimbal pedestal


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mars_rover")

    white_panel = model.material("white_panel", rgba=(0.92, 0.92, 0.90, 1.0))
    warm_white = model.material("warm_white", rgba=(0.88, 0.87, 0.84, 1.0))
    orange_accent = model.material("orange_accent", rgba=(0.93, 0.52, 0.12, 1.0))
    gray_metal = model.material("gray_metal", rgba=(0.55, 0.56, 0.58, 1.0))
    light_gray = model.material("light_gray", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    black_tread = model.material("black_tread", rgba=(0.06, 0.06, 0.07, 1.0))
    lens_dark = model.material("lens_dark", rgba=(0.04, 0.04, 0.05, 1.0))

    # ------------------------------------------------------------------
    # Body: white boxy chassis + equipment deck + rocker-bogie suspension
    # ------------------------------------------------------------------
    body = model.part("body")
    body.visual(
        Box(BODY_SIZE), origin=Origin(xyz=BODY_CENTER), material=white_panel, name="chassis_box"
    )
    body.visual(
        Box(DECK_SIZE), origin=Origin(xyz=DECK_CENTER), material=warm_white, name="deck_plate"
    )

    # Deck equipment (all seated on the deck plate).
    body.visual(
        Box((0.50, 0.50, 0.14)),
        origin=Origin(xyz=(-0.40, -0.25, DECK_TOP + 0.07)),
        material=white_panel,
        name="deck_equipment_box",
    )
    body.visual(
        Box((0.35, 0.34, 0.12)),
        origin=Origin(xyz=(-0.45, 0.30, DECK_TOP + 0.06)),
        material=warm_white,
        name="deck_electronics_box",
    )
    body.visual(
        Box((0.25, 0.18, 0.10)),
        origin=Origin(xyz=(0.38, -0.40, DECK_TOP + 0.05)),
        material=orange_accent,
        name="deck_orange_module",
    )
    body.visual(
        Box((0.20, 0.20, 0.12)),
        origin=Origin(xyz=(0.45, 0.45, DECK_TOP + 0.06)),
        material=gray_metal,
        name="deck_instrument_box",
    )
    body.visual(
        Box((0.30, 0.20, 0.05)),
        origin=Origin(xyz=(0.08, -0.08, DECK_TOP + 0.025)),
        material=lens_dark,
        name="deck_vent_grille",
    )

    # Front hazard-camera pods on the front chassis face.
    for cam_y, cam_name in ((0.25, "front_hazcam_left"), (-0.25, "front_hazcam_right")):
        body.visual(
            Box((0.05, 0.10, 0.08)),
            origin=Origin(xyz=(1.215, cam_y, 0.78)),
            material=lens_dark,
            name=cam_name,
        )

    # Rear UHF antenna: short pole + dark cylinder on the rear deck.
    body.visual(
        Cylinder(radius=0.020, length=0.24),
        origin=Origin(xyz=(-0.75, 0.45, DECK_TOP + 0.12)),
        material=gray_metal,
        name="uhf_antenna_pole",
    )
    body.visual(
        Cylinder(radius=0.050, length=0.20),
        origin=Origin(xyz=(-0.75, 0.45, DECK_TOP + 0.34)),
        material=dark_gray,
        name="uhf_antenna_can",
    )

    # High-gain antenna gimbal pedestal on the deck.
    body.visual(
        Box((0.14, 0.14, 0.10)),
        origin=Origin(xyz=(HGA_BASE[0], HGA_BASE[1], DECK_TOP + 0.05)),
        material=light_gray,
        name="hga_pedestal",
    )

    # Robotic-arm shoulder bracket on the front-right chassis face.
    body.visual(
        Box((0.12, 0.16, 0.22)),
        origin=Origin(xyz=(1.24, ARM_SHOULDER[1], ARM_SHOULDER[2])),
        material=warm_white,
        name="arm_shoulder_bracket",
    )

    # Rocker-bogie suspension, mirrored per side; every link chains
    # body -> rocker pivot -> rocker tubes -> (knuckle | bogie pivot ->
    # bogie tubes -> knuckle) -> axle stub -> wheel hub.
    for side, side_name in ((1.0, "left"), (-1.0, "right")):
        px, py, pz = ROCKER_PIVOT[0], side * ROCKER_PIVOT[1], ROCKER_PIVOT[2]
        bx, by, bz = BOGIE_PIVOT[0], side * BOGIE_PIVOT[1], BOGIE_PIVOT[2]

        # Differential pivot hub embedded into the chassis side wall.
        body.visual(
            Cylinder(radius=0.060, length=0.12),
            origin=Origin(xyz=(px, py, pz), rpy=(pi / 2.0, 0.0, 0.0)),
            material=light_gray,
            name=f"{side_name}_rocker_pivot_hub",
        )
        # Rocker front leg: pivot down to the front steering knuckle.
        body.visual(
            _mesh(
                f"{side_name}_rocker_front_leg.obj",
                tube_from_spline_points(
                    [
                        (px, py, pz),
                        (0.70, side * 0.86, 0.78),
                        (WHEEL_ROW_X[0], side * AXLE_INNER_Y, 0.55),
                    ],
                    radius=SUSP_TUBE_R,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=gray_metal,
            name=f"{side_name}_rocker_front_leg",
        )
        # Rocker rear leg: pivot back to the bogie pivot.
        body.visual(
            _mesh(
                f"{side_name}_rocker_rear_leg.obj",
                tube_from_spline_points(
                    [
                        (px, py, pz),
                        (-0.12, side * 0.78, 0.80),
                        (bx, by, bz),
                    ],
                    radius=SUSP_TUBE_R,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=gray_metal,
            name=f"{side_name}_rocker_rear_leg",
        )
        # Bogie pivot hub.
        body.visual(
            Cylinder(radius=0.052, length=0.10),
            origin=Origin(xyz=(bx, by, bz), rpy=(pi / 2.0, 0.0, 0.0)),
            material=light_gray,
            name=f"{side_name}_bogie_pivot_hub",
        )
        # Bogie front leg: bogie pivot forward-down to the middle axle.
        body.visual(
            _mesh(
                f"{side_name}_bogie_front_leg.obj",
                tube_from_spline_points(
                    [
                        (bx, by, bz),
                        (-0.18, side * 0.78, 0.42),
                        (WHEEL_ROW_X[1], side * AXLE_INNER_Y, WHEEL_RADIUS),
                    ],
                    radius=SUSP_TUBE_R * 0.9,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=gray_metal,
            name=f"{side_name}_bogie_front_leg",
        )
        # Bogie rear leg: bogie pivot back to the rear steering knuckle.
        body.visual(
            _mesh(
                f"{side_name}_bogie_rear_leg.obj",
                tube_from_spline_points(
                    [
                        (bx, by, bz),
                        (-0.72, side * 0.76, 0.58),
                        (WHEEL_ROW_X[2], side * AXLE_INNER_Y, 0.55),
                    ],
                    radius=SUSP_TUBE_R * 0.9,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=gray_metal,
            name=f"{side_name}_bogie_rear_leg",
        )
        # Vertical steering knuckle posts over the front and rear axles.
        for row_idx, row_name in ((0, "front"), (2, "rear")):
            body.visual(
                Cylinder(radius=0.040, length=0.31),
                origin=Origin(xyz=(WHEEL_ROW_X[row_idx], side * AXLE_INNER_Y, 0.405)),
                material=light_gray,
                name=f"{row_name}_{side_name}_steering_knuckle",
            )
        # Axle stubs from the suspension into each wheel hub.
        axle_len = AXLE_TIP_Y - (AXLE_INNER_Y - 0.02)
        for row_idx, row_name in enumerate(WHEEL_ROWS):
            axle_mid_y = side * (AXLE_INNER_Y - 0.02 + axle_len / 2.0)
            body.visual(
                Cylinder(radius=AXLE_RADIUS, length=axle_len),
                origin=Origin(
                    xyz=(WHEEL_ROW_X[row_idx], axle_mid_y, WHEEL_RADIUS),
                    rpy=(pi / 2.0, 0.0, 0.0),
                ),
                material=dark_gray,
                name=f"{row_name}_{side_name}_axle",
            )

    # ------------------------------------------------------------------
    # Six cleated wheels (shared meshes, one loop, parameterized stations)
    # ------------------------------------------------------------------
    tire_mesh = _mesh(
        "rover_tire.obj",
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.19,
            tread=TireTread(style="block", depth=0.014, count=24, land_ratio=0.55),
            sidewall=TireSidewall(style="square", bulge=0.02),
        ),
    )
    wheel_disc_mesh = _mesh(
        "rover_wheel_disc.obj",
        WheelGeometry(
            0.195,
            0.16,
            hub=WheelHub(radius=0.060, width=0.14, cap_style="domed"),
            face=WheelFace(dish_depth=0.010, front_inset=0.008, rear_inset=0.006),
        ),
    )

    for i in range(6):
        row = i % 3
        side = 1.0 if i < 3 else -1.0
        side_name = "left" if side > 0 else "right"
        wheel_name = f"{WHEEL_ROWS[row]}_{side_name}_wheel"

        wheel = model.part(wheel_name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, side * pi / 2.0)),
            material=black_tread,
            name="tire",
        )
        wheel.visual(
            wheel_disc_mesh,
            origin=Origin(rpy=(0.0, 0.0, side * pi / 2.0)),
            material=dark_gray,
            name="wheel_disc",
        )
        # Web plate bridging the hub to the rim/tire seat (keeps the wheel
        # a single connected assembly).
        wheel.visual(
            Cylinder(radius=0.165, length=0.030),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_gray,
            name="wheel_web",
        )
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(WHEEL_ROW_X[row], side * WHEEL_TRACK_Y, WHEEL_RADIUS)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=45.0, velocity=6.0),
        )

    # ------------------------------------------------------------------
    # Camera mast (yaw) + stereo camera head (tilt)
    # ------------------------------------------------------------------
    mast = model.part("camera_mast")
    mast.visual(
        Cylinder(radius=0.070, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=light_gray,
        name="mast_base_flange",
    )
    mast.visual(
        Cylinder(radius=0.045, length=0.84),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=gray_metal,
        name="mast_column",
    )
    mast.visual(
        Cylinder(radius=0.016, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.38), rpy=(pi / 2.0, 0.0, 0.0)),
        material=light_gray,
        name="mast_cross_handle",
    )
    mast.visual(
        Box((0.09, 0.07, 0.10)),
        origin=Origin(xyz=(0.05, 0.0, 0.60)),
        material=warm_white,
        name="mast_electronics_pod",
    )
    mast.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT - 0.03)),
        material=light_gray,
        name="mast_head_stub",
    )
    model.articulation(
        "mast_yaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=mast,
        origin=Origin(xyz=MAST_BASE),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-2.8, upper=2.8),
    )

    head = model.part("mast_camera_head")
    head.visual(
        Cylinder(radius=0.050, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=gray_metal,
        name="head_neck_collar",
    )
    head.visual(
        Box((0.30, 0.20, 0.16)),
        origin=Origin(xyz=(0.02, 0.0, 0.09)),
        material=white_panel,
        name="head_box",
    )
    for lens_y, lens_name in ((0.06, "head_lens_left"), (-0.06, "head_lens_right")):
        head.visual(
            Cylinder(radius=0.032, length=0.030),
            origin=Origin(xyz=(0.175, lens_y, 0.10), rpy=(0.0, pi / 2.0, 0.0)),
            material=lens_dark,
            name=lens_name,
        )
    head.visual(
        Box((0.03, 0.08, 0.03)),
        origin=Origin(xyz=(0.165, 0.0, 0.025)),
        material=orange_accent,
        name="head_accent_bar",
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=head,
        # Origin is expressed in the mast's local frame (mast frame sits at MAST_BASE).
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-0.45, upper=0.45),
    )

    # ------------------------------------------------------------------
    # Robotic arm with dark instrument turret (shoulder yaw)
    # ------------------------------------------------------------------
    arm = model.part("robotic_arm")
    arm.visual(
        Cylinder(radius=0.055, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=gray_metal,
        name="shoulder_hub",
    )
    arm.visual(
        _mesh(
            "arm_upper_link.obj",
            tube_from_spline_points(
                [(0.0, 0.0, 0.0), (0.26, 0.0, 0.09), (0.50, 0.0, 0.12)],
                radius=0.034,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=warm_white,
        name="upper_arm_link",
    )
    arm.visual(
        Cylinder(radius=0.050, length=0.10),
        origin=Origin(xyz=(0.50, 0.0, 0.12), rpy=(pi / 2.0, 0.0, 0.0)),
        material=light_gray,
        name="elbow_hub",
    )
    arm.visual(
        _mesh(
            "arm_forearm_link.obj",
            tube_from_spline_points(
                [(0.50, 0.0, 0.12), (0.72, 0.0, 0.27), (0.88, 0.0, 0.38)],
                radius=0.030,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=warm_white,
        name="forearm_link",
    )
    arm.visual(
        Cylinder(radius=0.045, length=0.09),
        origin=Origin(xyz=(0.88, 0.0, 0.38), rpy=(pi / 2.0, 0.0, 0.0)),
        material=light_gray,
        name="wrist_hub",
    )
    arm.visual(
        Cylinder(radius=0.095, length=0.16),
        origin=Origin(xyz=(0.99, 0.0, 0.38), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="instrument_turret",
    )
    arm.visual(
        Cylinder(radius=0.022, length=0.10),
        origin=Origin(xyz=(1.11, 0.0, 0.38), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_dark,
        name="turret_drill_bit",
    )
    arm.visual(
        Box((0.10, 0.06, 0.06)),
        origin=Origin(xyz=(0.99, 0.0, 0.49)),
        material=gray_metal,
        name="turret_sensor_block",
    )
    model.articulation(
        "arm_shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=arm,
        origin=Origin(xyz=ARM_SHOULDER),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.8, lower=-0.35, upper=0.50),
    )

    # ------------------------------------------------------------------
    # High-gain antenna dish on gimbal pedestal (azimuth)
    # ------------------------------------------------------------------
    hga = model.part("high_gain_antenna")
    hga.visual(
        Cylinder(radius=0.032, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=light_gray,
        name="hga_gimbal_stem",
    )
    dish_profile = [
        (0.015, 0.000),
        (0.140, 0.028),
        (0.280, 0.095),
        (0.280, 0.115),
        (0.140, 0.048),
        (0.030, 0.022),
        (0.015, 0.030),
        (0.015, 0.000),
    ]
    hga.visual(
        _mesh("hga_dish.obj", LatheGeometry(dish_profile, segments=48)),
        origin=Origin(xyz=(0.0, 0.0, 0.20), rpy=(0.42, 0.0, 0.0)),
        material=gray_metal,
        name="hga_dish",
    )
    hga.visual(
        Cylinder(radius=0.030, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.235), rpy=(0.42, 0.0, 0.0)),
        material=light_gray,
        name="hga_dish_boss",
    )
    model.articulation(
        "hga_azimuth",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hga,
        origin=Origin(xyz=HGA_BASE),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-2.6, upper=2.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")

    # --- rover identity: six wheels on a rocker-bogie, mast, arm, dish ---
    wheel_names = [
        f"{row}_{side}_wheel" for side in ("left", "right") for row in WHEEL_ROWS
    ]
    wheels = [object_model.get_part(name) for name in wheel_names]
    ctx.check("rover has six wheels", len(wheels) == 6, details=f"wheels={wheel_names}")

    for name in wheel_names:
        spin = object_model.get_articulation(f"{name}_spin")
        ctx.check(
            f"{name} spin joint is continuous",
            spin.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={spin.articulation_type}",
        )
        aabb = ctx.part_world_aabb(name)
        ctx.check(
            f"{name} rests on the ground plane",
            aabb is not None and abs(aabb[0][2]) <= 0.02,
            details=f"aabb={aabb}",
        )

    mast = object_model.get_part("camera_mast")
    head = object_model.get_part("mast_camera_head")
    arm = object_model.get_part("robotic_arm")
    hga = object_model.get_part("high_gain_antenna")

    for joint_name in ("mast_yaw", "head_tilt", "arm_shoulder_yaw", "hga_azimuth"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={joint.articulation_type}",
        )

    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "camera head sits high on the mast above the deck",
        head_aabb is not None and head_aabb[0][2] > DECK_TOP + 0.70,
        details=f"head_aabb={head_aabb}",
    )
    ctx.expect_overlap(
        head,
        mast,
        axes="xy",
        min_overlap=0.02,
        name="camera head stays carried by the mast column",
    )

    # Arm turret reaches forward of the chassis front face.
    arm_aabb = ctx.part_world_aabb(arm)
    ctx.check(
        "robotic arm turret reaches forward of the chassis",
        arm_aabb is not None and arm_aabb[1][0] > 1.30 + 0.60,
        details=f"arm_aabb={arm_aabb}",
    )

    # Shoulder yaw actually swings the turret sideways.
    rest_aabb = ctx.part_world_aabb(arm)
    shoulder = object_model.get_articulation("arm_shoulder_yaw")
    with ctx.pose({shoulder: 0.45}):
        swung_aabb = ctx.part_world_aabb(arm)
    ctx.check(
        "shoulder yaw swings the arm laterally",
        rest_aabb is not None
        and swung_aabb is not None
        and (swung_aabb[1][1] - rest_aabb[1][1]) > 0.15,
        details=f"rest={rest_aabb}, swung={swung_aabb}",
    )

    # Dish stays clear of the mast through its azimuth sweep.
    hga_joint = object_model.get_articulation("hga_azimuth")
    with ctx.pose({hga_joint: 2.2}):
        ctx.expect_gap(
            mast,
            hga,
            axis="x",
            min_gap=0.02,
            name="antenna dish clears the camera mast when slewed",
        )

    # --- intentional captured-fit overlaps -------------------------------
    for name in wheel_names:
        row, side = name.split("_")[0], name.split("_")[1]
        ctx.allow_overlap(
            body,
            name,
            elem_a=f"{row}_{side}_axle",
            elem_b="wheel_disc",
            reason="Drive axle stub is intentionally captured inside the wheel hub.",
        )
    ctx.allow_overlap(
        body,
        mast,
        reason="Mast base flange is seated into the deck plate mounting boss.",
    )
    ctx.allow_overlap(
        mast,
        head,
        reason="Head neck collar captures the mast head stub for the tilt bearing.",
    )
    ctx.allow_overlap(
        body,
        arm,
        reason="Shoulder hub is captured inside the front shoulder bracket bearing.",
    )
    ctx.allow_overlap(
        body,
        hga,
        reason="Gimbal stem is seated into the deck pedestal bearing bore.",
    )

    return ctx.report()


object_model = build_object_model()
