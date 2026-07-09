from __future__ import annotations

"""Apollo Lunar Roving Vehicle (LRV) — rear three-quarter view seed.

Modeled after the reference photo: open flat-deck chassis on four wire-mesh
wheels with dark chevron tread, tan dust fenders on struts, a rear equipment
rack with tool pallet behind the two webbed seats, a center console with a
tilting T-handle controller, and the umbrella-shaped high-gain antenna dish
on a swiveling forward mast, plus a low-gain antenna staff and TV camera.

Frame convention: +X forward, +Y left, +Z up, ground at z=0.
Articulations: 4x CONTINUOUS wheel spin, REVOLUTE antenna elevation pivot,
REVOLUTE hand-controller tilt, FIXED rear rack.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
WHEEL_RADIUS = 0.41  # tire outer radius; axle height = ground contact at z=0
WHEEL_WIDTH = 0.23
AXLE_Z = WHEEL_RADIUS
WHEEL_X = 1.145  # half wheelbase (2.29 m)
WHEEL_Y = 0.915  # half track (1.83 m)

DECK_TOP = 0.445
FENDER_R_IN = 0.44
FENDER_R_OUT = 0.475
FENDER_WIDTH = 0.30

# name -> (x, y) of each wheel center
WHEEL_POSITIONS = {
    "front_left": (WHEEL_X, WHEEL_Y),
    "front_right": (WHEEL_X, -WHEEL_Y),
    "rear_left": (-WHEEL_X, WHEEL_Y),
    "rear_right": (-WHEEL_X, -WHEEL_Y),
}

DISH_TILT = -0.5  # pitch of the umbrella dish (leans back toward Earth)


def _fender_profile() -> list[tuple[float, float]]:
    """Annular arch sector in XY (later rotated into the XZ wheel plane)."""
    a0, a1 = math.radians(-15.0), math.radians(195.0)
    n = 28
    pts: list[tuple[float, float]] = []
    for i in range(n + 1):
        a = a0 + (a1 - a0) * i / n
        pts.append((FENDER_R_OUT * math.cos(a), FENDER_R_OUT * math.sin(a)))
    for i in range(n + 1):
        a = a1 - (a1 - a0) * i / n
        pts.append((FENDER_R_IN * math.cos(a), FENDER_R_IN * math.sin(a)))
    return pts


def _dish_axis_point(dist: float) -> tuple[float, float, float]:
    """Point on the tilted dish axis, `dist` above the dish apex (antenna-local)."""
    return (
        math.sin(DISH_TILT) * dist,
        0.0,
        0.87 + math.cos(DISH_TILT) * dist,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apollo_lunar_roving_vehicle")

    alum = Material(name="alum_frame", rgba=(0.72, 0.73, 0.76, 1.0))
    mesh_silver = Material(name="mesh_silver", rgba=(0.82, 0.83, 0.86, 1.0))
    tread_dark = Material(name="tread_dark", rgba=(0.16, 0.16, 0.18, 1.0))
    fender_tan = Material(name="fender_tan", rgba=(0.80, 0.62, 0.38, 1.0))
    webbing_tan = Material(name="webbing_tan", rgba=(0.85, 0.78, 0.62, 1.0))
    box_black = Material(name="box_black", rgba=(0.11, 0.11, 0.12, 1.0))
    dish_bright = Material(name="dish_bright", rgba=(0.87, 0.87, 0.90, 1.0))

    # ------------------------------------------------------------- chassis
    chassis = model.part("chassis")

    # Flat open deck: floor pan + side rails + cross rails.
    chassis.visual(
        Box((2.6, 1.44, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=alum,
        name="floor_pan",
    )
    for side, sy in (("left", 1.0), ("right", -1.0)):
        chassis.visual(
            Box((2.6, 0.06, 0.10)),
            origin=Origin(xyz=(0.0, sy * 0.69, 0.42)),
            material=alum,
            name=f"side_rail_{side}",
        )
    for i, cx in enumerate((-1.0, 0.0, 1.0)):
        chassis.visual(
            Box((0.06, 1.38, 0.08)),
            origin=Origin(xyz=(cx, 0.0, 0.40)),
            material=alum,
            name=f"cross_rail_{i}",
        )

    # Shared mesh for the four tan fender arches.
    fender_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(_fender_profile(), FENDER_WIDTH),
        "lrv_fender_arch",
    )

    # Suspension arms, axle stubs and tan fenders per wheel station.
    for wname, (wx, wy) in WHEEL_POSITIONS.items():
        s = 1.0 if wy > 0 else -1.0
        chassis.visual(
            Box((0.10, 0.105, 0.06)),
            origin=Origin(xyz=(wx, s * 0.7375, AXLE_Z)),
            material=alum,
            name=f"suspension_arm_{wname}",
        )
        # Axle stub embeds into the spinning hub (axisymmetric captured shaft).
        chassis.visual(
            Cylinder(radius=0.03, length=0.14),
            origin=Origin(xyz=(wx, s * 0.85, AXLE_Z), rpy=(math.pi / 2, 0.0, 0.0)),
            material=alum,
            name=f"axle_{wname}",
        )
        # Fender arch over the wheel (profile XY -> wheel plane XZ via roll +90deg).
        chassis.visual(
            fender_mesh,
            origin=Origin(xyz=(wx, wy, AXLE_Z), rpy=(math.pi / 2, 0.0, 0.0)),
            material=fender_tan,
            name=f"fender_{wname}",
        )
        # Fender mount: vertical post off the side rail + lateral stay into the arch.
        chassis.visual(
            Box((0.05, 0.05, 0.44)),
            origin=Origin(xyz=(wx, s * 0.72, 0.665)),
            material=alum,
            name=f"fender_post_{wname}",
        )
        chassis.visual(
            Box((0.05, 0.26, 0.05)),
            origin=Origin(xyz=(wx, s * 0.85, 0.875)),
            material=alum,
            name=f"fender_stay_{wname}",
        )

    # Two webbed seats with backrests (rear of deck, visible seatbacks in photo).
    for side, sy in (("left", 1.0), ("right", -1.0)):
        chassis.visual(
            Box((0.45, 0.42, 0.04)),
            origin=Origin(xyz=(-0.15, sy * 0.35, 0.72)),
            material=webbing_tan,
            name=f"seat_pan_{side}",
        )
        chassis.visual(
            Box((0.05, 0.42, 0.52)),
            origin=Origin(xyz=(-0.385, sy * 0.35, 0.98)),
            material=webbing_tan,
            name=f"seat_back_{side}",
        )
        for j, (lx, ly) in enumerate(((0.18, 0.17), (0.18, -0.17), (-0.18, 0.17), (-0.18, -0.17))):
            chassis.visual(
                Box((0.04, 0.04, 0.26)),
                origin=Origin(xyz=(-0.15 + lx, sy * 0.35 + ly, 0.5725)),
                material=alum,
                name=f"seat_leg_{side}_{j}",
            )

    # Center control console with display panel.
    chassis.visual(
        Box((0.30, 0.25, 0.35)),
        origin=Origin(xyz=(0.30, 0.0, 0.6175)),
        material=box_black,
        name="console_body",
    )
    chassis.visual(
        Box((0.16, 0.22, 0.02)),
        origin=Origin(xyz=(0.24, 0.0, 0.80)),
        material=mesh_silver,
        name="console_panel",
    )

    # Forward equipment: blanketed battery / LCRU boxes.
    for side, sy in (("left", 1.0), ("right", -1.0)):
        chassis.visual(
            Box((0.34, 0.44, 0.18)),
            origin=Origin(xyz=(0.80, sy * 0.34, 0.53)),
            material=webbing_tan,
            name=f"battery_box_{side}",
        )

    # Low-gain antenna staff rising off the right-front equipment box.
    chassis.visual(
        Cylinder(radius=0.012, length=0.55),
        origin=Origin(xyz=(0.85, -0.45, 0.72)),
        material=alum,
        name="lowgain_mast",
    )
    chassis.visual(
        Cylinder(radius=0.025, length=0.10),
        origin=Origin(xyz=(0.85, -0.45, 0.95)),
        material=box_black,
        name="lowgain_head",
    )

    # TV camera unit on a short forward post.
    chassis.visual(
        Cylinder(radius=0.015, length=0.30),
        origin=Origin(xyz=(1.18, -0.12, 0.595)),
        material=alum,
        name="camera_post",
    )
    chassis.visual(
        Box((0.12, 0.12, 0.14)),
        origin=Origin(xyz=(1.18, -0.12, 0.79)),
        material=box_black,
        name="camera_body",
    )
    chassis.visual(
        Cylinder(radius=0.03, length=0.05),
        origin=Origin(xyz=(1.25, -0.12, 0.80), rpy=(0.0, math.pi / 2, 0.0)),
        material=box_black,
        name="camera_lens",
    )

    # -------------------------------------------------------------- wheels
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.30,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
            tread=TireTread(style="chevron", depth=0.014, count=24, angle_deg=30.0, land_ratio=0.55),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.012, radius=0.006),
        ),
        "lrv_tire",
    )
    disc_mesh = mesh_from_geometry(
        WheelGeometry(
            0.31,
            0.20,
            rim=WheelRim(
                inner_radius=0.27,
                flange_height=0.012,
                flange_thickness=0.006,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(radius=0.06, width=0.14, cap_style="domed"),
            face=WheelFace(dish_depth=0.02, front_inset=0.01, rear_inset=0.01),
            spokes=WheelSpokes(style="split_y", count=8, thickness=0.012, window_radius=0.04),
        ),
        "lrv_wheel_disc",
    )

    for wname, (wx, wy) in WHEEL_POSITIONS.items():
        wheel = model.part(f"wheel_{wname}")
        # Wheel helpers spin about local X; yaw +90deg aligns them to the Y axle.
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
            material=tread_dark,
            name="tire",
        )
        wheel.visual(
            disc_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
            material=mesh_silver,
            name="wheel_disc",
        )
        model.articulation(
            f"chassis_to_wheel_{wname}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=wheel,
            origin=Origin(xyz=(wx, wy, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=10.0),
        )

    # ---------------------------------------------------- rear equipment rack
    rack = model.part("rear_equipment_rack")
    # Local frame at the rear deck edge, on the deck top plane.
    for side, sy in (("left", 1.0), ("right", -1.0)):
        rack.visual(
            Box((0.30, 0.05, 0.04)),
            origin=Origin(xyz=(0.08, sy * 0.45, 0.015)),
            material=alum,
            name=f"rack_base_rail_{side}",
        )
        rack.visual(
            Box((0.05, 0.05, 0.55)),
            origin=Origin(xyz=(-0.05, sy * 0.45, 0.29)),
            material=alum,
            name=f"rack_post_{side}",
        )
        rack.visual(
            Box((0.08, 0.05, 0.05)),
            origin=Origin(xyz=(-0.09, sy * 0.45, 0.455)),
            material=alum,
            name=f"rack_pallet_stub_{side}",
        )
    rack.visual(
        Box((0.30, 0.95, 0.04)),
        origin=Origin(xyz=(0.08, 0.0, 0.015)),
        material=alum,
        name="rack_base_rail_center",
    )
    rack.visual(
        Box((0.05, 0.95, 0.05)),
        origin=Origin(xyz=(-0.05, 0.0, 0.555)),
        material=alum,
        name="rack_top_rail",
    )
    # Tool pallet plate hanging off the rear of the rack.
    rack.visual(
        Box((0.04, 0.95, 0.50)),
        origin=Origin(xyz=(-0.12, 0.0, 0.355)),
        material=alum,
        name="tool_pallet_plate",
    )
    # Hand-tool handles racked on the pallet, poking above it.
    for i, ty in enumerate((-0.25, 0.0, 0.25)):
        rack.visual(
            Cylinder(radius=0.012, length=0.45),
            origin=Origin(xyz=(-0.14, ty, 0.615)),
            material=box_black,
            name=f"tool_handle_{i}",
        )
    # Sample collection bag hooked on the pallet.
    rack.visual(
        Box((0.10, 0.18, 0.22)),
        origin=Origin(xyz=(-0.07, -0.30, 0.495)),
        material=webbing_tan,
        name="sample_bag",
    )
    model.articulation(
        "chassis_to_rear_rack",
        ArticulationType.FIXED,
        parent=chassis,
        child=rack,
        origin=Origin(xyz=(-1.30, 0.0, DECK_TOP)),
    )

    # ------------------------------------------------------ high-gain antenna
    hga = model.part("high_gain_antenna")
    hga.visual(
        Cylinder(radius=0.02, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=alum,
        name="hga_mast",
    )
    hga.visual(
        Box((0.06, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
        material=box_black,
        name="hga_gimbal",
    )
    # Umbrella wire-mesh dish: thin revolved shell, tilted back toward Earth.
    dish_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.015, 0.0), (0.10, 0.032), (0.20, 0.072), (0.31, 0.125)],
            [(0.015, 0.007), (0.10, 0.039), (0.20, 0.079), (0.31, 0.132)],
            segments=40,
        ),
        "hga_dish",
    )
    hga.visual(
        dish_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.87), rpy=(0.0, DISH_TILT, 0.0)),
        material=dish_bright,
        name="hga_dish",
    )
    # Hub stub ties the dish apex down into the gimbal block.
    hga.visual(
        Cylinder(radius=0.02, length=0.12),
        origin=Origin(xyz=_dish_axis_point(-0.03), rpy=(0.0, DISH_TILT, 0.0)),
        material=alum,
        name="hga_hub_stub",
    )
    # Feed rod + feed tip along the dish axis.
    hga.visual(
        Cylinder(radius=0.007, length=0.30),
        origin=Origin(xyz=_dish_axis_point(0.15), rpy=(0.0, DISH_TILT, 0.0)),
        material=alum,
        name="hga_feed_rod",
    )
    hga.visual(
        Sphere(radius=0.02),
        origin=Origin(xyz=_dish_axis_point(0.30)),
        material=box_black,
        name="hga_feed_tip",
    )
    model.articulation(
        "chassis_to_high_gain_antenna",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=hga,
        origin=Origin(xyz=(1.05, 0.35, DECK_TOP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.7, upper=1.0),
    )

    # -------------------------------------------------- T-handle hand controller
    controller = model.part("hand_controller")
    controller.visual(
        Cylinder(radius=0.015, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=box_black,
        name="controller_stem",
    )
    controller.visual(
        Box((0.04, 0.14, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=box_black,
        name="controller_grip",
    )
    model.articulation(
        "chassis_to_hand_controller",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=controller,
        origin=Origin(xyz=(0.40, 0.0, 0.78)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    rack = object_model.get_part("rear_equipment_rack")
    hga = object_model.get_part("high_gain_antenna")
    controller = object_model.get_part("hand_controller")
    elev = object_model.get_articulation("chassis_to_high_gain_antenna")
    tilt = object_model.get_articulation("chassis_to_hand_controller")

    # --- intentional embeddings -------------------------------------------
    for wname in WHEEL_POSITIONS:
        ctx.allow_overlap(
            chassis,
            object_model.get_part(f"wheel_{wname}"),
            elem_a=f"axle_{wname}",
            elem_b="wheel_disc",
            reason="Axisymmetric axle stub is intentionally captured inside the spinning hub.",
        )
    for side in ("left", "right"):
        ctx.allow_overlap(
            chassis,
            rack,
            elem_a="floor_pan",
            elem_b=f"rack_base_rail_{side}",
            reason="Rack base rails seat 5 mm into the deck pan as a bolted mount proxy.",
        )
    ctx.allow_overlap(
        chassis,
        rack,
        elem_a="floor_pan",
        elem_b="rack_base_rail_center",
        reason="Center rack rail seats 5 mm into the deck pan as a bolted mount proxy.",
    )
    ctx.allow_overlap(
        chassis,
        hga,
        elem_a="floor_pan",
        elem_b="hga_mast",
        reason="Antenna mast root is intentionally socketed into the deck pan.",
    )
    ctx.allow_overlap(
        chassis,
        controller,
        elem_a="console_body",
        elem_b="controller_stem",
        reason="Controller stem is a captured pivot shaft inside the console proxy.",
    )

    # --- four spinning mesh wheels on the ground ---------------------------
    for wname in WHEEL_POSITIONS:
        wheel = object_model.get_part(f"wheel_{wname}")
        joint = object_model.get_articulation(f"chassis_to_wheel_{wname}")
        jt = joint.articulation_type
        ctx.check(
            f"wheel joint {wname} is continuous spin",
            jt == ArticulationType.CONTINUOUS or str(jt).lower().endswith("continuous"),
            details=f"type={jt}",
        )
        ctx.check(
            f"wheel joint {wname} spins about the lateral Y axle",
            abs(joint.axis[1]) > 0.99,
            details=f"axis={joint.axis}",
        )
        aabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"wheel {wname} touches the lunar surface",
            aabb is not None and abs(aabb[0][2]) <= 0.02,
            details=f"aabb={aabb}",
        )
        # Tan fender covers the wheel from above.
        ctx.expect_overlap(
            chassis,
            wheel,
            elem_a=f"fender_{wname}",
            axes="xy",
            min_overlap=0.12,
            name=f"fender covers wheel {wname} in plan view",
        )
        fender_aabb = ctx.part_element_world_aabb(chassis, elem=f"fender_{wname}")
        ctx.check(
            f"fender {wname} arches above the wheel crown",
            fender_aabb is not None and aabb is not None and fender_aabb[1][2] > aabb[1][2] + 0.02,
            details=f"fender={fender_aabb}, wheel={aabb}",
        )

    # --- rear equipment rack / tool pallet (hero of this photo) ------------
    ctx.expect_origin_gap(
        chassis,
        rack,
        axis="x",
        min_gap=1.0,
        name="equipment rack hangs off the rear of the chassis",
    )
    rack_aabb = ctx.part_world_aabb(rack)
    ctx.check(
        "rear rack sits fully behind the rear axle area",
        rack_aabb is not None and rack_aabb[1][0] < -1.0,
        details=f"aabb={rack_aabb}",
    )
    ctx.check(
        "tool handles rise above the pallet",
        rack_aabb is not None and rack_aabb[1][2] > 1.15,
        details=f"aabb={rack_aabb}",
    )
    rack.get_visual("tool_pallet_plate")
    rack.get_visual("sample_bag")

    # --- seats with backrests ----------------------------------------------
    for side in ("left", "right"):
        pan = ctx.part_element_world_aabb(chassis, elem=f"seat_pan_{side}")
        back = ctx.part_element_world_aabb(chassis, elem=f"seat_back_{side}")
        ctx.check(
            f"seat backrest {side} rises above the seat pan",
            pan is not None and back is not None and back[1][2] > pan[1][2] + 0.3,
            details=f"pan={pan}, back={back}",
        )

    # --- umbrella high-gain antenna elevation pivot -------------------------
    dish_aabb = ctx.part_element_world_aabb(hga, elem="hga_dish")
    ctx.check(
        "umbrella dish rides high on the forward mast",
        dish_aabb is not None and dish_aabb[0][2] > 1.1,
        details=f"dish={dish_aabb}",
    )
    # Verify the joint axis is lateral Y (elevation), not vertical Z (azimuth).
    ctx.check(
        "chassis_to_high_gain_antenna pivots about lateral Y for elevation",
        abs(elev.axis[1]) > 0.99,
        details=f"axis={elev.axis}",
    )
    rest_center_z = None
    if dish_aabb is not None:
        rest_center_z = (dish_aabb[0][2] + dish_aabb[1][2]) / 2.0
    with ctx.pose({elev: 0.8}):
        posed = ctx.part_element_world_aabb(hga, elem="hga_dish")
        posed_center_z = None
        if posed is not None:
            posed_center_z = (posed[0][2] + posed[1][2]) / 2.0
        tilted = (
            rest_center_z is not None
            and posed_center_z is not None
            and abs(posed_center_z - rest_center_z) > 0.02
        )
        ctx.check(
            "elevation pivot changes dish height at positive q",
            tilted,
            details=f"rest_z={rest_center_z}, posed_z={posed_center_z}",
        )

    # --- T-handle controller tilts forward/back ----------------------------
    grip_rest = ctx.part_element_world_aabb(controller, elem="controller_grip")
    with ctx.pose({tilt: 0.35}):
        grip_posed = ctx.part_element_world_aabb(controller, elem="controller_grip")
        moved = (
            grip_rest is not None
            and grip_posed is not None
            and abs(grip_posed[0][0] - grip_rest[0][0]) > 0.03
        )
        ctx.check(
            "hand controller grip tilts along the driving axis",
            moved,
            details=f"rest={grip_rest}, posed={grip_posed}",
        )

    return ctx.report()


object_model = build_object_model()
