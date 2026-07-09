"""Mars Exploration Rover (Spirit/Opportunity class) solar-panel rover.

Identity features from the reference picture:
- low warm-gray electronics body topped by a large flat winged deck of dark
  blue-black solar panels with a visible cell grid
- tall silver pancam mast at the deck front-center with a pan/tilt stereo
  camera head bar
- six small dark cleated wheels on silver rocker-bogie suspension arms
- jointed instrument arm tucked under the front of the body
- high-gain antenna panel and low-gain antenna whip on the deck

Articulation: 6 continuous wheel axles, mast head pan, camera bar tilt,
instrument-arm shoulder pitch.
"""

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- layout ----
WHEEL_RADIUS = 0.130
WHEEL_WIDTH = 0.100
AXLE_Z = 0.130          # wheel centers -> tires touch ground z=0
WHEEL_Y = 0.420         # wheel center lateral offset
STRUT_Y = 0.340         # suspension strut plane (clear of wheel inner face)

FRONT_X = 0.34
MID_X = -0.08
REAR_X = -0.44
ROCKER_PIVOT = (0.02, 0.40)   # (x, z) rocker pivot on the body side
BOGIE_PIVOT = (-0.20, 0.32)   # (x, z) bogie pivot on the rocker arm

BODY_TOP_Z = 0.52
DECK_Z0 = 0.515               # solar deck slab bottom (embeds body top)
DECK_Z1 = 0.535               # solar deck slab top
GRID_Z = 0.5355               # grid lines sit just proud of the deck top

MAST_XY = (0.28, 0.0)
MAST_TOP_Z = 1.08

WHEEL_SIDES = (("left", 1.0), ("right", -1.0))
WHEEL_STATIONS = (("front", FRONT_X), ("middle", MID_X), ("rear", REAR_X))


def _strut(part, p0, p1, radius, material, name):
    """Add a cylinder visual whose axis runs from point p0 to point p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    yaw = math.atan2(dy, dx)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mer_solar_panel_mars_rover")

    solar_cell = model.material("solar_cell", rgba=(0.08, 0.09, 0.16, 1.0))
    panel_grid = model.material("panel_grid", rgba=(0.72, 0.74, 0.78, 1.0))
    body_tan = model.material("body_tan", rgba=(0.62, 0.58, 0.50, 1.0))
    silver = model.material("suspension_silver", rgba=(0.72, 0.73, 0.75, 1.0))
    mast_silver = model.material("mast_silver", rgba=(0.78, 0.79, 0.81, 1.0))
    brass = model.material("brass_ring", rgba=(0.55, 0.45, 0.25, 1.0))
    wheel_dark = model.material("wheel_dark", rgba=(0.16, 0.16, 0.17, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.38, 0.38, 0.40, 1.0))
    hardware = model.material("dark_hardware", rgba=(0.22, 0.22, 0.24, 1.0))
    lens_black = model.material("lens_black", rgba=(0.05, 0.05, 0.06, 1.0))

    # ------------------------------------------------------------ chassis --
    chassis = model.part("chassis")

    # Warm Electronics Box body.
    chassis.visual(
        Box((0.90, 0.60, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=body_tan,
        name="body_box",
    )
    # Front hazcam dots on the body face.
    for hy in (-0.10, 0.10):
        chassis.visual(
            Cylinder(radius=0.015, length=0.024),
            origin=Origin(xyz=(0.452, hy, 0.36), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=lens_black,
            name=f"hazcam_{'left' if hy > 0 else 'right'}",
        )

    # -------- winged solar deck: center slab + front nose + two rear wings.
    deck_zc = (DECK_Z0 + DECK_Z1) / 2.0
    chassis.visual(
        Box((1.00, 0.66, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, deck_zc)),
        material=solar_cell,
        name="solar_deck_center",
    )
    chassis.visual(
        Box((0.30, 0.44, 0.02)),
        origin=Origin(xyz=(0.60, 0.0, deck_zc)),
        material=solar_cell,
        name="solar_deck_nose",
    )
    wing_specs = (("left", 1.0, math.radians(150.0)), ("right", -1.0, math.radians(-150.0)))
    for wname, wsign, wyaw in wing_specs:
        cx = -0.645
        cy = 0.39 * wsign
        chassis.visual(
            Box((0.52, 0.36, 0.02)),
            origin=Origin(xyz=(cx, cy, deck_zc), rpy=(0.0, 0.0, wyaw)),
            material=solar_cell,
            name=f"solar_wing_{wname}",
        )
        # Cell grid lines along the wing.
        for k, t in enumerate((-0.10, 0.0, 0.10)):
            px = cx - math.sin(wyaw) * t
            py = cy + math.cos(wyaw) * t
            chassis.visual(
                Box((0.48, 0.005, 0.004)),
                origin=Origin(xyz=(px, py, GRID_Z), rpy=(0.0, 0.0, wyaw)),
                material=panel_grid,
                name=f"wing_grid_{wname}_{k}",
            )

    # Cell grid on the center deck: lengthwise + crosswise silver lines.
    for i, gy in enumerate((-0.22, -0.11, 0.0, 0.11, 0.22)):
        chassis.visual(
            Box((0.98, 0.005, 0.004)),
            origin=Origin(xyz=(0.0, gy, GRID_Z)),
            material=panel_grid,
            name=f"deck_grid_long_{i}",
        )
    for i in range(7):
        gx = -0.40 + i * (0.80 / 6.0)
        chassis.visual(
            Box((0.005, 0.64, 0.004)),
            origin=Origin(xyz=(gx, 0.0, GRID_Z)),
            material=panel_grid,
            name=f"deck_grid_cross_{i}",
        )
    for i, gx in enumerate((0.52, 0.60, 0.68)):
        chassis.visual(
            Box((0.005, 0.42, 0.004)),
            origin=Origin(xyz=(gx, 0.0, GRID_Z)),
            material=panel_grid,
            name=f"nose_grid_{i}",
        )
    # Silver deck border frame.
    for i, by in enumerate((-0.325, 0.325)):
        chassis.visual(
            Box((1.00, 0.010, 0.005)),
            origin=Origin(xyz=(0.0, by, GRID_Z)),
            material=panel_grid,
            name=f"deck_edge_y_{i}",
        )
    for i, bx in enumerate((-0.495, 0.495)):
        chassis.visual(
            Box((0.010, 0.66, 0.005)),
            origin=Origin(xyz=(bx, 0.0, GRID_Z)),
            material=panel_grid,
            name=f"deck_edge_x_{i}",
        )

    # -------- pancam mast tower (static part of the mast).
    chassis.visual(
        Box((0.10, 0.10, 0.04)),
        origin=Origin(xyz=(MAST_XY[0], MAST_XY[1], 0.55)),
        material=mast_silver,
        name="mast_base",
    )
    chassis.visual(
        Cylinder(radius=0.032, length=0.53),
        origin=Origin(xyz=(MAST_XY[0], MAST_XY[1], 0.815)),
        material=mast_silver,
        name="mast_tube",
    )
    chassis.visual(
        Cylinder(radius=0.038, length=0.06),
        origin=Origin(xyz=(MAST_XY[0], MAST_XY[1], 1.01)),
        material=brass,
        name="mast_collar",
    )

    # -------- high-gain antenna panel + low-gain antenna whip on the deck.
    chassis.visual(
        Cylinder(radius=0.015, length=0.17),
        origin=Origin(xyz=(-0.25, -0.18, 0.615)),
        material=mast_silver,
        name="hga_post",
    )
    chassis.visual(
        Box((0.20, 0.20, 0.015)),
        origin=Origin(xyz=(-0.25, -0.18, 0.71), rpy=(0.0, 0.35, 0.0)),
        material=hardware,
        name="hga_panel",
    )
    chassis.visual(
        Cylinder(radius=0.008, length=0.37),
        origin=Origin(xyz=(-0.05, 0.20, 0.715)),
        material=mast_silver,
        name="lga_whip",
    )
    chassis.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.05, 0.20, 0.90)),
        material=mast_silver,
        name="lga_tip",
    )

    # -------- instrument-arm shoulder bracket under the body front.
    chassis.visual(
        Box((0.10, 0.06, 0.08)),
        origin=Origin(xyz=(0.42, 0.14, 0.26)),
        material=hardware,
        name="arm_shoulder_bracket",
    )

    # ----------------------------------------------- rocker-bogie + wheels --
    tire_geom = TireGeometry(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        inner_radius=0.090,
        tread=TireTread(style="block", depth=0.010, count=18, land_ratio=0.55),
        sidewall=TireSidewall(style="square", bulge=0.02),
        shoulder=TireShoulder(width=0.008, radius=0.003),
    )
    tire_mesh = mesh_from_geometry(tire_geom, "rover_tire")

    for side_name, sy in WHEEL_SIDES:
        ys = STRUT_Y * sy
        pivot_a = (ROCKER_PIVOT[0], ys, ROCKER_PIVOT[1])
        pivot_b = (BOGIE_PIVOT[0], ys, BOGIE_PIVOT[1])

        # Body-side mounting bracket for the rocker pivot (on the chassis).
        chassis.visual(
            Box((0.10, 0.08, 0.08)),
            origin=Origin(xyz=(ROCKER_PIVOT[0], 0.33 * sy, ROCKER_PIVOT[1])),
            material=body_tan,
            name=f"rocker_bracket_{side_name}",
        )

        # Rocker: pivot hub, forward arm to the front wheel, rear arm to bogie.
        rocker = model.part(f"rocker_{side_name}")
        rocker.visual(
            Cylinder(radius=0.045, length=0.035),
            origin=Origin(xyz=pivot_a, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name="rocker_hub",
        )
        _strut(rocker, pivot_a, (FRONT_X, ys, AXLE_Z), 0.018, silver, "rocker_front_strut")
        _strut(rocker, pivot_a, pivot_b, 0.018, silver, "rocker_rear_strut")
        rocker.visual(
            Cylinder(radius=0.016, length=0.11),
            origin=Origin(xyz=(FRONT_X, ys, AXLE_Z + 0.055)),
            material=silver,
            name="front_knuckle_post",
        )
        rocker.visual(
            Box((0.05, 0.04, 0.04)),
            origin=Origin(xyz=(FRONT_X, ys, AXLE_Z + 0.11)),
            material=hardware,
            name="front_steer_motor",
        )
        model.articulation(
            f"chassis_to_rocker_{side_name}",
            ArticulationType.FIXED,
            parent=chassis,
            child=rocker,
        )

        # Bogie: pivot hub plus arms to the middle and rear wheels.
        bogie = model.part(f"bogie_{side_name}")
        bogie.visual(
            Cylinder(radius=0.040, length=0.030),
            origin=Origin(xyz=pivot_b, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name="bogie_hub",
        )
        _strut(bogie, pivot_b, (MID_X, ys, AXLE_Z), 0.018, silver, "bogie_front_strut")
        _strut(bogie, pivot_b, (REAR_X, ys, AXLE_Z), 0.018, silver, "bogie_rear_strut")
        bogie.visual(
            Cylinder(radius=0.016, length=0.11),
            origin=Origin(xyz=(REAR_X, ys, AXLE_Z + 0.055)),
            material=silver,
            name="rear_knuckle_post",
        )
        bogie.visual(
            Box((0.05, 0.04, 0.04)),
            origin=Origin(xyz=(REAR_X, ys, AXLE_Z + 0.11)),
            material=hardware,
            name="rear_steer_motor",
        )
        model.articulation(
            f"rocker_to_bogie_{side_name}",
            ArticulationType.FIXED,
            parent=rocker,
            child=bogie,
        )

        # Six cleated wheels: front on the rocker, middle + rear on the bogie.
        for station_name, wx in WHEEL_STATIONS:
            carrier = rocker if station_name == "front" else bogie
            carrier.visual(
                Cylinder(radius=0.020, length=0.10),
                origin=Origin(xyz=(wx, 0.38 * sy, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=silver,
                name=f"axle_stub_{station_name}",
            )
            wheel = model.part(f"wheel_{side_name}_{station_name}")
            wheel.visual(
                tire_mesh,
                origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
                material=wheel_dark,
                name="wheel_tire",
            )
            wheel.visual(
                Cylinder(radius=0.091, length=0.056),
                origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=hub_gray,
                name="wheel_face",
            )
            wheel.visual(
                Cylinder(radius=0.032, length=0.080),
                origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=mast_silver,
                name="wheel_hub_boss",
            )
            model.articulation(
                f"axle_{side_name}_{station_name}",
                ArticulationType.CONTINUOUS,
                parent=carrier,
                child=wheel,
                origin=Origin(xyz=(wx, WHEEL_Y * sy, AXLE_Z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=40.0, velocity=8.0),
            )

    # ----------------------------------------------------- pancam mast head --
    mast_head = model.part("mast_head")
    mast_head.visual(
        Cylinder(radius=0.028, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=mast_silver,
        name="head_neck",
    )
    mast_head.visual(
        Box((0.035, 0.030, 0.05)),
        origin=Origin(xyz=(0.04, 0.0, 0.04)),
        material=hardware,
        name="pan_motor",
    )
    model.articulation(
        "mast_pan",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=mast_head,
        origin=Origin(xyz=(MAST_XY[0], MAST_XY[1], MAST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-3.0, upper=3.0),
    )

    camera_bar = model.part("camera_bar")
    camera_bar.visual(
        Box((0.09, 0.34, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=hardware,
        name="bar_body",
    )
    for eye_name, ey in (("left", 0.12), ("right", -0.12)):
        camera_bar.visual(
            Cylinder(radius=0.022, length=0.05),
            origin=Origin(xyz=(0.06, ey, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=mast_silver,
            name=f"camera_eye_{eye_name}",
        )
        camera_bar.visual(
            Cylinder(radius=0.015, length=0.008),
            origin=Origin(xyz=(0.088, ey, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=lens_black,
            name=f"camera_lens_{eye_name}",
        )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=mast_head,
        child=camera_bar,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.7, upper=0.7),
    )

    # ------------------------------------------------------ instrument arm --
    arm = model.part("instrument_arm")
    arm.visual(
        Cylinder(radius=0.035, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="arm_shoulder_hub",
    )
    _strut(arm, (0.0, 0.0, 0.0), (0.14, 0.0, -0.05), 0.020, silver, "arm_upper_link")
    arm.visual(
        Cylinder(radius=0.030, length=0.045),
        origin=Origin(xyz=(0.14, 0.0, -0.05), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="arm_elbow",
    )
    _strut(arm, (0.14, 0.0, -0.05), (0.02, 0.0, -0.10), 0.018, silver, "arm_forearm")
    arm.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(0.02, 0.0, -0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="arm_turret",
    )
    arm.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.075, 0.0, -0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="turret_spectrometer",
    )
    arm.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.02, 0.0, -0.155)),
        material=silver,
        name="turret_rat_tool",
    )
    model.articulation(
        "arm_shoulder",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=arm,
        origin=Origin(xyz=(0.45, 0.14, 0.24)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.0, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")

    # ---------------------------------------------- intentional embeddings --
    for side_name, _sy in WHEEL_SIDES:
        rocker = f"rocker_{side_name}"
        bogie = f"bogie_{side_name}"
        for elem in ("rocker_hub", "rocker_front_strut", "rocker_rear_strut"):
            ctx.allow_overlap(
                "chassis",
                rocker,
                elem_a=f"rocker_bracket_{side_name}",
                elem_b=elem,
                reason="Rocker pivot hub and strut roots are captured inside the body-side mounting bracket.",
            )
        for elem in ("bogie_hub", "bogie_front_strut", "bogie_rear_strut"):
            ctx.allow_overlap(
                rocker,
                bogie,
                elem_a="rocker_rear_strut",
                elem_b=elem,
                reason="Bogie pivot hub and strut roots are captured on the rocker rear-arm pivot pin.",
            )
        for station_name, _wx in WHEEL_STATIONS:
            carrier = rocker if station_name == "front" else bogie
            for elem in ("wheel_face", "wheel_hub_boss"):
                ctx.allow_overlap(
                    carrier,
                    f"wheel_{side_name}_{station_name}",
                    elem_a=f"axle_stub_{station_name}",
                    elem_b=elem,
                    reason="Drive axle stub is intentionally captured inside the wheel hub.",
                )

    ctx.allow_overlap(
        "chassis",
        "mast_head",
        elem_a="mast_tube",
        elem_b="head_neck",
        reason="Pan bearing: the head neck is seated inside the top of the mast tube.",
    )
    ctx.allow_overlap(
        "mast_head",
        "camera_bar",
        elem_a="head_neck",
        elem_b="bar_body",
        reason="Tilt bearing: the camera bar saddle seats on the head neck top.",
    )
    ctx.allow_overlap(
        "chassis",
        "instrument_arm",
        elem_a="arm_shoulder_bracket",
        elem_b="arm_shoulder_hub",
        reason="Shoulder pivot hub is captured inside the front bracket clevis.",
    )
    ctx.allow_overlap(
        "chassis",
        "instrument_arm",
        elem_a="arm_shoulder_bracket",
        elem_b="arm_upper_link",
        reason="Upper-arm root passes through the shoulder bracket opening at the stowed pose.",
    )

    # ------------------------------------------------------- wheel checks --
    wheel_joints = []
    for side_name, _sy in WHEEL_SIDES:
        for station_name, _wx in WHEEL_STATIONS:
            wheel_joints.append(f"axle_{side_name}_{station_name}")
    ctx.check("six wheel axles authored", len(wheel_joints) == 6, details=str(wheel_joints))
    for joint_name in wheel_joints:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=str(joint.articulation_type),
        )
    wheel_parts = [p for p in object_model.parts if p.name.startswith("wheel_")]
    ctx.check(
        "six wheel parts authored",
        len(wheel_parts) == 6,
        details=str([p.name for p in wheel_parts]),
    )
    for part in wheel_parts:
        aabb = ctx.part_world_aabb(part)
        ctx.check(
            f"{part.name} rests on the ground plane",
            aabb is not None and -0.01 <= aabb[0][2] <= 0.02,
            details=f"aabb={aabb}",
        )

    # -------------------------------------------------- mast + deck checks --
    pan = object_model.get_articulation("mast_pan")
    tilt = object_model.get_articulation("head_tilt")
    shoulder = object_model.get_articulation("arm_shoulder")
    ctx.check("mast pan is revolute", pan.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("head tilt is revolute", tilt.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("arm shoulder is revolute", shoulder.articulation_type == ArticulationType.REVOLUTE)

    deck = ctx.part_element_world_aabb(chassis, elem="solar_deck_center")
    ctx.check(
        "solar deck spans the body top",
        deck is not None
        and (deck[1][0] - deck[0][0]) >= 0.90
        and deck[0][2] >= BODY_TOP_Z - 0.02,
        details=f"deck={deck}",
    )
    for wname in ("left", "right"):
        wing = ctx.part_element_world_aabb(chassis, elem=f"solar_wing_{wname}")
        ctx.check(f"solar wing {wname} present", wing is not None, details=str(wing))
    mast = ctx.part_element_world_aabb(chassis, elem="mast_tube")
    ctx.check(
        "pancam mast rises above one meter",
        mast is not None and mast[1][2] >= 1.0,
        details=f"mast={mast}",
    )

    # ------------------------------------------------ decisive pose checks --
    def _center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    camera_bar = object_model.get_part("camera_bar")
    eye_rest = ctx.part_element_world_aabb(camera_bar, elem="camera_eye_left")
    with ctx.pose({pan: 1.2}):
        eye_panned = ctx.part_element_world_aabb(camera_bar, elem="camera_eye_left")
    ctx.check(
        "mast pan swings the camera head",
        eye_rest is not None
        and eye_panned is not None
        and math.hypot(
            _center(eye_panned)[0] - _center(eye_rest)[0],
            _center(eye_panned)[1] - _center(eye_rest)[1],
        )
        > 0.03,
        details=f"rest={eye_rest}, panned={eye_panned}",
    )
    with ctx.pose({tilt: 0.6}):
        eye_tilted = ctx.part_element_world_aabb(camera_bar, elem="camera_eye_left")
    ctx.check(
        "head tilt pitches the camera eyes",
        eye_rest is not None
        and eye_tilted is not None
        and abs(_center(eye_tilted)[2] - _center(eye_rest)[2]) > 0.015,
        details=f"rest={eye_rest}, tilted={eye_tilted}",
    )

    arm = object_model.get_part("instrument_arm")
    turret_rest = ctx.part_element_world_aabb(arm, elem="arm_turret")
    with ctx.pose({shoulder: 1.0}):
        turret_up = ctx.part_element_world_aabb(arm, elem="arm_turret")
    ctx.check(
        "positive shoulder motion deploys the arm upward",
        turret_rest is not None
        and turret_up is not None
        and _center(turret_up)[2] > _center(turret_rest)[2] + 0.03,
        details=f"rest={turret_rest}, deployed={turret_up}",
    )

    return ctx.report()


object_model = build_object_model()
